import numpy as np

#establish internode structure 

class node:
    def __init__(self, state, previous, added_order): 
        #list of numbers associated with pancake positions 
        self.state = state
        #previous node
        self.previous = previous
        #number of previous nodes 
        self.added_order = added_order
        #must pre-establish path_cost
        self.backward_cost = 0
    
    
    #create path cost function
    def forward_cost(self): 
        heuristic_gap = 0
        previous_p = self.state[0]
        for p in self.state[1:]: 
            if np.absolute(p-previous_p) != 1:
                heuristic_gap +=1 
            previous_p = p
        return heuristic_gap
    
    #cost will be determined in terms of numbers of pancakes flipped
    def pancake_number_flipped(self, number_flipped):
        self.number_flipped = number_flipped
        #reverse order of pancakes at point flip point, must account for even and odd scenarios
        for i in range(int(number_flipped / 2)): 
            anything = self.state[i]
            self.state[i] = self.state[number_flipped-i-1]
            self.state[number_flipped-i-1] = anything
        #backwards cost is the sum of the flips 
        self.backward_cost += number_flipped    
        
    #sum forward and backwards cost 
    def total_cost(self):
        return self.forward_cost() + self.backward_cost
        
    #priorities cheapest total cost, depth first search in the case of a tie   
    def __lt__(self, other):
        if self.total_cost() != other.total_cost():
            return self.total_cost() < other.total_cost()
        else: 
            self.added_order < other.added_order
        
import heapq 

class Priority(): 
    #a heap queue 
    def __init__(self): 
        self.heap_queue = []
    
    #mount node onto the priority queue
    def mount(self, node): 
        heapq.heappush(self.heap_queue, node)
    
        #account for fact that multiple nodes have the same state
    def same_state_nodes(self, state):
        for i in self.heap_queue: 
            if i.state == state:
                return True
            else: 
                return False
    
    #account for node ordering incorrectness during search 
    def update_ordering(self, cheaper_node): 
        for i in self.heap_queue: 
            if i.state == cheaper_node.state: 
                if cheaper_node.total_cost() < i.total_cost(): 
                    self.heap[self.heap_queue.index(i)] = cheaper_node
    #pop cheapest from the heap
    def pop_cheapest(self):
        heap_min = heapq.heappop(self.heap_queue);
        return heap_min
                    
    #determine if queue is complete
    def complete(self):
        return len(self.heap_queue) == 0

#runs and returns A* 
import copy

class A_algorithm(): 
    #initialize algorithm 
    def __init__(self, initial_state): 
        #include priority node 
        self.expansion_implementation = Priority()
        #don't need to initialize initial state, because it is user input
        self.length = len(initial_state)
        #blocked nodes are nodes that have already been exmained 
        self.blocked = []
        self.added_order = 0
        #establish order of operations 
        self.conf = node(initial_state, None, self.added_order)
        self.expansion_implementation.mount(self.conf)
        self.added_order += 1
    
    #all states must have plate on bottom, and be comprised include all numbers 1-plate
    def constraints(self):
        pancake_stack = self.conf.state
        for p in pancake_stack:
            if p > pancake_stack[-1]: 
                exit()
        if sorted(pancake_stack) != list(range(min(pancake_stack), max(pancake_stack)+1)):
            exit()
    
    #frontier is comprised of all nodes reachable by current node
    #children show possible stack after flip 
    #flip depth can't be one nor can it include the plate
    def frontier_expansion(self, node_current):
        for number_flipped in range(2, self.length):
            child = copy.deepcopy(node_current) 
            child.pancake_number_flipped(number_flipped)
            child.previous = node_current
            child.added_order = self.added_order
            
            #if chlid state not blocked and not in frontier, place in frontier 
            if (child.state not in self.blocked) and (not self.expansion_implementation.same_state_nodes(child.state)):
                self.expansion_implementation.mount(child)
            
            #always opt for cheaper node
            elif self.expansion_implementation.same_state_nodes(child.state): 
                self.expansion_implementation.update_ordering
                
            self.added_order += 1

    
    #execute A* algorithm 
    def execute(self): 
        while True: 
            #if not complete and goal not found, then no solution
            if self.expansion_implementation.complete(): 
                self.solution = False
                return
            
            #node implementation, take cheapest from heap
            node_current = self.expansion_implementation.pop_cheapest()
            #block state from being used in the future
            self.blocked.append(node_current.state)
            
            # If the heuristic function returns 0, then we are at the goal
            if node_current.forward_cost() == 0:
                self.solution = node_current
                return
            
            # Add every possible node that the current node can reach to the frontier.
            self.frontier_expansion(node_current)

    def show_solution(self):
        #just in case
        if self.solution == False: 
            return print("Failure")
        
        else:
            #create node chain
            node_chain = []
            node_current = self.solution
            while node_current != None: 
                node_chain.append(node_current)
                node_current = node_current.previous
            
            # If there is only one state in our solution, then the user input
            # is already the solution
            if len(node_chain) == 1:
                print("Initial State = Goal State")
                exit()
            
            # Reverse the steps because it's backwards
            node_chain.reverse()
            
            # Finally, print the steps to get to the solution
            print("Initial Pancake Stack: ", node_chain[0].state)
            print("A* search:")
            for successor in range(1, len(node_chain)):
                print("Flip", successor, ":", node_chain[successor].number_flipped,
                    "pancakes")
                print("New State = ", node_chain[successor].state)

a_alg = A_algorithm([9,8,7,6,5,4,3,1,2,10])
a_alg.constraints()
a_alg.execute()
a_alg.show_solution()
        