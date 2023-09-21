#lang dssl2

# Final project: Trip Planner

import cons
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'
import sbox_hash
#import hw2-5
### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segm rents are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

struct position: 
    let lat : Lat?
    let long : Lon?
    
struct info:
    let cat: Cat?
    let name: Name?
    
### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let _pois  #vector of raw POI
    let _plen: num? #length of pois
    let _len #number of positions
    let _graph  #position node - u, end segment - v, distance - w
    let _position_index #hashtable of positions with indexes
    let _index_position
    let _position_info
    
    #takes in vector of raw road segments, vector or raw POI
    def __init__(self, segments: VecC[RawSeg?], pois: VecC[RawPOI?]):
        self._pois = pois
        self._plen = pois.len() #length of poi
        self._len = 0 #number of positions
        self._graph = WUGraph(segments.len()*2) #position node - u, end segment - v, distance - w
        self._position_index = HashTable(segments.len()*2, make_sbox_hash())
        self._index_position = HashTable(segments.len()*2, make_sbox_hash())
        self._position_info = HashTable(segments.len()*2, make_sbox_hash()) # posn; listofInfo
        
        for r in segments:
            let x = r[0]
            let y = r[1]
            let x2 = r[2]
            let y2 = r[3]
            let dist = ((y2 - y) * (y2 - y) + (x2 - x)* (x2 - x)).sqrt()

            let p = position(x, y)
            let p2 = position(x2, y2)
            
            if not self._position_index.mem?(p):
                self._len = self._len + 1
                self._position_index.put(p, self._len - 1) #put functioon handles duplicates
                self._index_position.put(self._len - 1, p)
                
            if not self._position_index.mem?(p2):
                self._len = self._len + 1
                self._position_index.put(p2, self._len - 1)
                self._index_position.put(self._len - 1, p2)
            
            if self._graph.get_edge(self._position_index.get(p),self._position_index.get(p2)) is None:   
                self._graph.set_edge(self._position_index.get(p),self._position_index.get(p2), dist)
        
        for poi in pois:
            let pos = position(poi[0], poi[1])
            let i = info(poi[2], poi[3])
            if self._position_info.mem?(pos):
                let list = cons(i, self._position_info.get(pos))
                self._position_info.put(pos, list)
            else:
                self._position_info.put(pos, cons(i, None)) #key:pos, value:list of info                   
        
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        ): # ListC[RawPos?] # return list of positions of the POIs    
        let l = None 
        let i = 0
        
        while i < self._plen and self._pois[i] is not None : 
            if self._pois[i][2] == dst_cat:
                let rawpos = [self._pois[i][0], self._pois[i][1]]
                let present = False
                let list = l
                #println(list)
                while list is not None:
                    if list.data == rawpos: #how to compare positions
                        present = True 
                        break       
                    else:
                        list = list.next   
                if not present:
                    l = cons(rawpos, l)
            i = i + 1
        return l
   
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        ):
        let list = None
        let start = position(src_lat, src_lon)
        let start_node = self._position_index.get(start)

        let dest = None
        let i = 0
        while i < self._plen and self._pois[i] is not None:
            if self._pois[i][3] == dst_name:
                dest = position(self._pois[i][0], self._pois[i][1])
            i = i + 1
        if dest is None: #no destination
            #println('a')
            return None
        
        if start == dest:
            return cons([start.lat, start.long], None)
        #dijkstra's algorithm
        let table = self.dijkstra(self._graph, start_node)
        let dest_node = self._position_index.get(dest)
        #if destination unreachable, return None
        if table[0][dest_node] == inf:
            return None
        #start from end and trace back to start node
        list = cons([dest.lat, dest.long], list)
        
        let curr = table[1][dest_node]
        while curr is not start_node:
            let pos = self._index_position.get(curr)
            list = cons([pos.lat, pos.long], list)          
            pos = table[1][curr]
            curr = pos
        #if start_node is not dest_node:    
        list = cons([start.lat, start.long], list) 
        return list  # ->        ListC[RawPos?]  path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        ):   #->        ListC[RawPOI?]  list of nearby POIs    
        let list = None
        let start = position(src_lat, src_lon)
        let start_node = self._position_index.get(start)
        
        let table = self.dijkstra(self._graph, start_node)

        let dist = table[0]
        let bin_heap = BinHeap[nat?](self._len, lambda a, b : dist[a] < dist[b])
        
        let i = 0
        while i < self._len:
            #println('a')
            if dist[i] is not None and dist[i] is not inf:
                bin_heap.insert(i)
            i = i + 1 
        #println(bin_heap)  
        
        while n > 0 and bin_heap.len() > 0:

            let min = bin_heap.find_min()
            #println(min)
            bin_heap.remove_min()
            
            let pos = self._index_position.get(min)
            #println(pos)
            let info_list = None
            if self._position_info.mem?(pos):
                info_list = self._position_info.get(pos)

            while info_list is not None:
                let info = info_list.data
                if info.cat == dst_cat and n > 0:
                    let poi = [pos.lat, pos.long, info.cat, info.name]
                    list = cons(poi, list)
                    n = n - 1
                info_list = info_list.next

        return list

    #takes graph and starting node and runs the dijkstra aalgorithm. returns a table with the 
    #distance map and predecessors map    
    def dijkstra(self, graph, start) : # graph aand a starting vertex position
        let dist = vec(self._len)
        let pred = vec(self._len)
        let i = 0
        while i < self._len:
            dist[i] = inf
            pred[i] = None
            i = i+1
        
        dist[start] = 0
        let todo = BinHeap[nat?](self._len * self._len, lambda a, b : dist[a] < dist[b])#empty priority queue
        let done = [False; self._len]#empty vertex set
        
        todo.insert(start)
        
        while todo.len() > 0:
            let v = todo.find_min() #remove element of todo with minimal dist[v]
            todo.remove_min()
       
            if not done[v]:
                done[v] = True
                let alist = graph.get_adjacent(v)
                while alist is not None:
                    let u = alist.data
                    if dist[v] + graph.get_edge(v, u) < dist[u]:
                        dist[u] = dist[v] + graph.get_edge(v, u)
                        pred[u] = v
                        todo.insert(u)
                    alist = alist.next
        let table = [dist, pred]
        return table             
#   ^ YOUR CODE GOES HERE


def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)

def my_2nd_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0], [0,1,0,2], [0,1,0,0], [0,1,1,1],
                        [0,2,1,2], [1,2,1,1], [1,3,1,2], [1,2,1,3], [1,3,-.2,3.3], [-.2,3.3,1,3],
                        [1,1,1,0], [1,0,0,0], [1,0,1,1], [100,1,22,2]], #unreachable
                       [[0,1, "clothes", "Pants"],
                        [0,1, "food", "Pasta"], [0,0, "food", "Sandwiches"], [-.2,3.3, "food", "Burritos"],
                        [1,1, "bank", "Local Credit Union"], [1,3, "bar", "Bar None"], [1,3, "bar", "H Bar"],
                        [100, 1, "food", "A"]])
                        #add unreachable poi
                     
test 'all food':
    assert my_2nd_example().locate_all("food") == \
        cons([100,1], cons([-.2,3.3], cons([0,0], cons([0,1], None))))

test 'all banks':
    assert my_2nd_example().locate_all("bank") == \
        cons([1,1], None)  

test 'route a':
   assert my_2nd_example().plan_route(0, 0, "Sandwiches") == \
       cons([0,0], None)                     
test 'route b':
   assert my_2nd_example().plan_route(0, 1, "Sandwiches") == \
       cons([0,1], cons([0,0], None))
test 'route c':
   assert my_2nd_example().plan_route(1, 1, "Sandwiches") == \
       cons([1,1], cons([0,1], cons([0,0], None)))
test 'route d':
   assert my_2nd_example().plan_route(0, 1, "Sushi") == \
       None      
test 'route e':
   assert my_2nd_example().plan_route(1, 1, "A") == \
       None  
test 'nearby 1':
    assert my_2nd_example().find_nearby(1, 3, "food", 1) == \
        cons([-.2,3.3, "food", "Burritos"], None)     
test 'nearby 2':
    assert my_2nd_example().find_nearby(0, 2, "food", 1) == \
        cons([0,1, "food", "Pasta"], None)         
#test 'nearby 3':
#    assert my_2nd_example().find_nearby(0, 2, "food", 2) == \
#        cons([0,1, "food", "Pasta"], cons([0,0, "food", "Sandwiches"], None))           
#test 'nearby 4':
 #   assert my_2nd_example().find_nearby(0, 2, "food", 3) == \
  #      cons([-.2,3.3, "food", "Burritos"], cons([0,1, "food", "Pasta"], cons([0,0, "food", "Sandwiches"], None)))          
#test 'nearby 5':
 #   assert my_2nd_example().find_nearby(0, 2, "food", 4) == \
  #      cons([-.2,3.3, "food", "Burritos"], cons([0,1, "food", "Pasta"], cons([0,0, "food", "Sandwiches"], None)))          
test 'nearby 6':
    assert  my_2nd_example().find_nearby(0, 2, "bar", 1) == \
        cons([1,3, "bar", "H Bar"], None)                 
test 'nearby 7':
    assert my_2nd_example().find_nearby(0, 2, "bar", 2) == \
        cons([1,3, "bar", "Bar None"], cons([1,3, "bar", "H Bar"], None))        
test 'nearby 8':
    assert my_2nd_example().find_nearby(0, 2, "bar", 5) == \
        cons([1,3, "bar", "Bar None"], cons([1,3, "bar", "H Bar"], None))        
test 'nearby 6':
    assert my_2nd_example().find_nearby(0, 2, "school", 5) == \
         None        
test 'nearby 7':
    assert my_2nd_example().find_nearby(100, 1, "school", 5) == \
         None            
test 'g3':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) \
    == [[0, 0], [1, 0]]
    
test 'g1':    
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [2.5, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) \
      == [[0, 0], [1.5, 0], [2.5, 0]]
    
test 'g4':     
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) \
      == [[0, 0], [1.5, 0], [2.5, 0], [3, 0]]
test 'g2':        
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(3, 0, 'Union')
    assert Cons.to_vec(result) \
      == [[3, 0], [2.5, 0], [1.5, 0]]    
test 'g2':        
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[0, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) \
      == [[0, 0]]        
test 'g7': 
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) \
      == []        
test 'g8':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.plan_route(0, 0, 'Cem')
    assert Cons.to_vec(result) \
      == [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]]
test 'g9':
    let tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result = tp.plan_route(-1.1, -1.1, 'Tony')
    assert Cons.to_vec(result) \
      == [[-1.1, -1.1], [0, 0], [3, 4]]                
test '10':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) \
      == [[0, 0], [1.5, 0], [2.5, 0], [3, 0], [4, 0], [5, 0]] 
test '11':
    let tp = TripPlanner(
      [[-2, 0, 0, 2],
       [0, 2, 2, 0],
       [2, 0, 0, -2],
       [0, -2, -2, 0]],
      [[2, 0, 'cooper', 'Dennis']])
    let result = tp.plan_route(-2, 0, 'Dennis')
    assert Cons.to_vec(result) \
      == [[-2, 0], [0, -2], [2, 0]]           
test '12':
    let tp = TripPlanner(
      [[0, 0, 0, 1],
       [0, 1, 3, 0],
       [0, 1, 4, 0],
       [0, 1, 5, 0],
       [0, 1, 6, 0],
       [0, 0, 1, 1],
       [1, 1, 3, 0],
       [1, 1, 4, 0],
       [1, 1, 5, 0],
       [1, 1, 6, 0],
       [0, 0, 2, 1],
       [2, 1, 3, 0],
       [2, 1, 4, 0],
       [2, 1, 5, 0],
       [2, 1, 6, 0]],
      [[0, 0, 'blacksmith', "Revere's Silver Shop"],
       [6, 0, 'church', 'Old North Church']])
    let result = tp.plan_route(0, 0, 'Old North Church')
    assert Cons.to_vec(result) \
      == [[0, 0], [2, 1], [6, 0]] 
test '13':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
    let result = tp.find_nearby(0, 0, 'bank', 1)
    assert (Cons.to_vec(result)) \
      == [[1, 0, 'bank', 'Union']]
test '14':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.find_nearby(0, 0, 'barber', 1)
    assert (Cons.to_vec(result)) \
      == [[3, 0, 'barber', 'Tony']]
    let resul = tp.find_nearby(3, 0, 'bank', 1)
    assert Cons.to_vec(resul) \
      == [[1.5, 0, 'bank', 'Union']]
test '15':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert (Cons.to_vec(result)) \
      == [[3, 0, 'barber', 'Tony']]
test '16':    
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert (Cons.to_vec(result)) \
      == [] 
test '17':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert (Cons.to_vec(result)) \
      == []
test '18':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.find_nearby(0, 0, 'haberdasher', 2)
    assert (Cons.to_vec(result)) \
      == [[8, 8, 'haberdasher', 'Braden'], [7, 7, 'haberdasher', 'Archit']]  
test '19':
    let tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result = tp.find_nearby(-1.1, -1.1, 'barber', 1)
    assert (Cons.to_vec(result)) \
      == [[3, 4, 'barber', 'Tony']]
test '20': 
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 3)
    assert (Cons.to_vec(result)) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
test '21':
    let tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [3.5, 0, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    let result = tp.find_nearby(-1, -1, 'barber', 1)
    assert (Cons.to_vec(result)) \
      == [[3.5, 0, 'barber', 'Tony']]      

test '22':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert (Cons.to_vec(result)) \
      ==[[5, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]      
test '23':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [0, 0, 'barber', 'Lily'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(2.5, 0, 'barber', 2)
    assert (Cons.to_vec(result)) \
      ==[[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]      
test '24':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert (Cons.to_vec(result)) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]      
      
                          
let eight_principles = ["Know your rights.",
     "Acknowledge your sources.",
     "Protect your work.",
     "Avoid suspicion.",
     "Do your own work.",
     "Never falsify a record or permit another person to do so.",
     "Never fabricate data, citations, or experimental results.",
     "Always tell the truth when discussing your work with your instructor."]
