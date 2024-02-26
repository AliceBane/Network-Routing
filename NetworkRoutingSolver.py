# Alice Giola
# CS 4412
# Project 3 - Network Routing

#!/usr/bin/python3

from CS4412Graph import *
import time
import math

# This project gave me the most trouble out of any other so far. I had help from
# my boyfriend who helped me understand what the algorithm is supposed to do at
# each point of the program.

# NOTE!  I have changed a few lines in the graph file, which are marked with the
# comment “mine” next to them. I had permission from the professor to make modifications.

class NetworkRoutingSolver:
    def __init__( self):
        pass

    def initializeNetwork( self, network ):
        assert( type(network) == CS4412Graph )
        self.network = network

    # Finds and returns the shortest path to 'destIndex' from the previously computed source.
    # Time complexity: O(E) where E is the number of edges in the path.
    def getShortestPath( self, destIndex ):
        self.dest = destIndex
        node_set = self.network.getNodes()
        node_start = node_set[self.source]
        node_finish = node_set[self.dest]
        node_current = node_finish

        edges = []
        length_total = 0
        while node_current != node_start:
            for neighbor in node_current.previous.neighbors:
                if neighbor.dest == node_current:
                    edges.append((neighbor.src.loc, neighbor.dest.loc, '{:.0f}'.format(neighbor.length)))
                    length_total += neighbor.length
                    node_current = node_current.previous

        return {'cost': length_total, 'path': edges}

    # Find the shortest paths from 'srcIndex' to all other nodes using Dijkstra's algorithm.
    # Time complexity: O(n^2) for array PQ, O((V+E)log(V)) for heap PQ, where V is vertices and E is edges.
    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()

        # Lets the program know which form of priority queue it should be using for calculations.
        if use_heap == True:
            queue_selection = PriorityQueueHeap()
        else:
            queue_selection = PriorityQueueArray()

        # Initiates the initial node set, so that they all start with no previous connections to
        # one another and starts all distances at infinity. Time complexity: O(n) where
        # n is the number of nodes in the graph).
        self.network.nodes[self.source].setDistance(0)
        for node in self.network.getNodes():
            if node != self.network.nodes[self.source]: node.setDistance(math.inf)
            node.setPrevious(None)
            queue_selection.insert(node)

        # An implementation of Dijkstra's algorithm. Depending on the priority queue selected above, will
        # have either a time complexity of O(V^2) for the array priority queue and Big-O((V+E)log(V))
        # for the heap priority queue.
        while queue_selection.is_empty() == False:
            node = queue_selection.pop_min()
            for neighbor in node.neighbors:
                if neighbor.dest.distance > (node.distance + neighbor.length):
                    queue_selection.decrease_key(neighbor.dest, node.distance + neighbor.length)
                    neighbor.dest.previous = node

        t2 = time.time()
        return (t2 - t1)

# My own implementation of an array priority queue that uses only what is
# necessary for the algorithm to function.
class PriorityQueueArray:
    def __init__(self):
        self.queue = []

    # Adds a node to the queue. Time complexity: O(1).
    def insert(self, node):
        self.queue.append(node)

    # Removes and returns the node with the minimum distance.
    # Time complexity: O(n) where n is the number of nodes in the queue.
    def pop_min(self):
        min_node = self.queue[0]
        for node in self.queue:
            if node.distance < min_node.distance: min_node = node
        self.queue.remove(min_node)
        return min_node

    # Updates the distance of a node. Time complexity: O(1)
    def decrease_key(self, node, priority):
        node.setDistance(priority)

    # Checks if the queue is empty. Time complexity: O(1)
    def is_empty(self):
        return len(self.queue) == 0

# This doesn't work when the starting node is 7 and the final node is 1 when in a graph of size 7. Not sure
# why this occurs or if there is a "natural" reason for this. I'm under the assumption
# that it is the programs way of reading these nodes in reverse and not finding a solution.
class PriorityQueueHeap:
    def __init__(self):
        self.heap = [0]
        self.node_map = {}

    def parent(self, i):
        return i // 2

    def left_child(self, i):
        return 2 * i

    def right_child(self, i):
        return (2 * i) + 1

    def swap(self, i, j):
        self.heap[i], self.heap[j] = self.heap[j], self.heap[i]

    # Inserts a node into the heap. Time complexity: O(log n).
    def insert(self, node):
        self.heap.append(node)
        self.percolate_up(len(self.heap) - 1)
        self.update(len(self.heap) - 1)

    # Decreases the distance of a node and reorders the heap. Time complexity: O(log n).
    def decrease_key(self, node, priority):
        temp = node.distance
        node.setDistance(priority)
        if priority < temp: self.percolate_up(self.node_map[node])

    # Compares a node with its children and returns the index of the smallest.
    # It's used in the percolate down process. Time complexity: O(1) for this operation,
    # but it's part of processes that are O(log n).
    def min_heapify(self, i):

        l_child = self.left_child(i)
        r_child = self.right_child(i)
        if r_child > len(self.heap) - 1: return l_child
        if self.heap[l_child].distance < self.heap[r_child].distance: return l_child
        return r_child

    # Moves a node up the heap until the heap property is restored.
    # This is used after insertions and decrease_key operations.
    # Time complexity: O(log n), as it may have to traverse up to the root.
    def percolate_up(self, i):
        while self.parent(i) > 0:
            if self.heap[self.parent(i)].distance > self.heap[i].distance:
                self.swap(self.parent(i), i)
                self.update(i)
                self.update(self.parent(i))
            i = self.parent(i)

    # Moves a node down the heap to maintain the heap property after removals.
    # Time complexity: O(log n), as it may have to traverse down to the leaf nodes.
    def percolate_down(self, i):
        while self.left_child(i) <= len(self.heap) - 1:
            min_temp = self.min_heapify(i)
            if self.heap[i].distance > self.heap[min_temp].distance:
                self.swap(i, min_temp)
                self.update(i)
                self.update(min_temp)
            i = min_temp

    # Removes and returns the node with the minimum distance.
    # Time complexity: O(log n) because of percolate_down.
    def pop_min(self):
        temp = self.heap[1]
        self.heap[1] = self.heap[len(self.heap) - 1]
        self.heap.pop()
        self.percolate_down(1)
        return temp

    # Checks if the heap is empty. Time complexity: O(1).
    def is_empty(self):
        return len(self.heap) - 1 == 0

    def update(self, i):
        self.node_map[self.heap[i]] = i
