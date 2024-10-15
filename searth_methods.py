import csv, time, math, tracemalloc, heapq
from collections import deque

#stores the cooridnates.csv file
coordinates = {}
with open('./coordinates.csv', newline='') as csvfile:
   #csv reader object is created
    csvreader = csv.reader(csvfile)


    #iterating through the csv to store the city name and coordinates
    for row in csvreader:
        city_name = row[0]
        latitude = float(row[1])
        longitude = float(row[2])

        coordinates[city_name] = (latitude, longitude)

adjacency = {}
with open('./Adjacencies.txt', 'r') as file:
    for line in file:

        city_one, city_two = line.strip().split(' ')

        #if the city doesn't exist in adjacency
        if city_one not in adjacency:
            #initialize an empty list for that city
            adjacency[city_one] = []
        #adds the adjacent city to the list of that city
        adjacency[city_one].append(city_two)

        if city_two not in adjacency:
            adjacency[city_two] = []
        adjacency[city_two].append(city_one)


#Finds the coordinates between a two coordinate locations
def coordinate_length(coordinate_one, coordinate_two):
    #the approximate radius of earth in kilometers
    radius_of_earth = 7371

    #Get and store the coordinates for each pair
    latitude_one, longitude_one = coordinate_one
    latitude_two, longitude_two = coordinate_two

    #convert latitude and longitude from degrees to radians
    #the math library in python uses radians so this conversion is necessary
    latitude_one, longitude_one, latitude_two, longitude_two = map(math.radians, [latitude_one, longitude_one, latitude_two, longitude_two])

    #Difference between the latitudes of the two separate pairs
    diff_latitude = latitude_two - latitude_one
    #Difference between the longitudes of the two separate pairs
    diff_longitude = longitude_two - longitude_one

    #Online I found the Haversine formula for calculating the length
    #The first line implements the first part of the equation and 
    #calulcates the angle between two points on a sphere
    angle_between_points = math.sin(diff_latitude/2)**2 + math.cos(latitude_one) * math.cos(latitude_two) * math.sin(diff_longitude/2)**2
    
    #The second part of the equation calculates the central angle between two points
    #but takes into account the sign of the inputs
    central_angle = 2 * math.atan2(math.sqrt(angle_between_points), math.sqrt(1-angle_between_points))

    #combines the two equations together
    distance = radius_of_earth * central_angle

    return distance




# Breadth First Search
# adjacency: list of towns with adjacent towns
# start: starting town picked by the user
# end: ending town picked by the user
def breadth_first_search(adjacency,coordinates, start, end):

    #memory tracking and timer initialization
    tracemalloc.start()
    start_time = time.time()

    queue = deque([[start]])#starts the queue with the starting city
    visited = set([start])#tracks the cities you've visited

    while queue:
        #get the first path from the queue
        path = queue.popleft()
        #get the last city in the current path which is the current city being explored
        current_city = path[-1]

        #check to see if the town is reached
        if current_city == end:

            total_distance = 0
            for i in range(len(path) - 1):
                #name of the first town reached
                town_one = path[i]
                #name of the second twon reached
                town_two = path[i+1]
                #calculate the distance between each one
                total_distance += coordinate_length(coordinates[town_one], coordinates[town_two])
            #end the timer
            end_time = time.time()
            #obtain the memory usage
            current, peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()#stop memory usage
            return path, total_distance, end_time - start_time, current, peak
        
        #explore the adjacent neighbors
        #if the neighbor hasn't been visited, add a new path to the queue
        for neighbor in adjacency.get(current_city, []):
            
            if neighbor not in visited:
                #If a neighbor has not been visited then it is assed to visited list
                visited.add(neighbor)
                #A new path is created and added to the queue
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
        
        end_time = time.time()
        current,peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()

    return None, 0, end_time - start_time, current, peak

# Depth First Search
# adjacency: list of towns with adjacent towns
# coordinates: coordinates of the towns
# start: starting town picked by the user
# end: ending town picked by the user
def depth_first_search(adjacency, coordinates, start, end):

    #initialize memory and timer
    tracemalloc.start()
    start_time = time.time()

    #stack to keep track of paths
    stack = [(start, [start], 0)]#[(current_city, path, distance_travaled)]
    #stack to keep track of visited cities
    visited = set()

    #continue as long as there are more paths
    while stack:

        #remove the last entry from the stack
        current_city, path, distance_traveled = stack.pop()

        #if the current city is the ending city
        if current_city == end:
            #end the time
            end_time = time.time()
            #get peak memory usage
            current,peak = tracemalloc.get_traced_memory()
            #stop using memory
            tracemalloc.stop()
            return path, distance_traveled, end_time-start_time, current, peak
        
        #adds to stack if not visited
        if current_city not in visited:
            visited.add(current_city)

            for neighbor in adjacency.get(current_city, []):
                if neighbor not in visited:
                    #calculates the distance between the current city and neighbor
                    distance = coordinate_length(coordinates[current_city], coordinates[neighbor])
                    #add the neighbor to the stack
                    stack.append((neighbor, path + [neighbor], distance_traveled + distance))
    
    end_time = time.time()
    current,peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return None, 0, end_time-start_time,current, peak


# Depth First Search
# adjacency: list of towns with adjacent towns
# coordinates: coordinates of the towns
# start: starting town picked by the user
# end: ending town picked by the user
# max_depth: maximum depth to search in the graph
def iddfs(adjacency,coordinates, start, end, max_depth):
    #start the memory allocation and timer
    tracemalloc.start()
    start_time = time.time()

    #loop from depth 0 to max_depth
    for depth in range(max_depth + 1):
        #stack used to keep track of paths
        stack = [(start, [start], set([start]), 0)]


        while stack:
            #current path being explored
            #path: list of cities in the current path from start to current city
            #visited: set of visited cities
            #total distance traveled
            current,path,visited, distance_traveled = stack.pop()

            #if current is at the end location then end all the timer and memory usage
            if current == end:
                end_time = time.time()
                current_mem, peak_mem = tracemalloc.get_traced_memory()
                tracemalloc.stop()
                return path, distance_traveled, end_time-start_time,current_mem,peak_mem
            
            #checks to make sure the depth hasn't been reached
            if len(path) - 1 < depth:
                #checks each neighbor of the current city in the adjacency list
                for neighbor in adjacency.get(current, []):
                    #calculate the coordinates and add to the list if not visited
                    if neighbor not in visited:
                        distance = coordinate_length(coordinates[current], coordinates[neighbor])
                        stack.append((neighbor, path+ [neighbor], visited | {neighbor}, distance_traveled + distance))

    end_time = time.time()
    current_mem,peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return None, 0, end_time - start_time, current_mem, peak_mem


# Best First Search
# adjacency: list of towns with adjacent towns
# coordinates: coordinates of the towns
# start: starting town picked by the user
# end: ending town picked by the user
def best_first_search(adjacency, coordinates, start, end):

    #memory tracking and timer
    tracemalloc.start()
    start_time = time.time()

    #First value is initial value cost
    #second value is the starting city
    #third is the initial starting city
    #fourth is the distance traveled
    priority_list = [(0,start,[start],0)]

    #set for visited towns which is set to 0
    visited = set()

    #While towns still exist
    while priority_list:
        #the current town being explored
        #path taken to reach current_town
        #distance traveled

        val, current_town, path,distance_traveled = heapq.heappop(priority_list)

        #if end is met then stop the timer and memory usage
        if current_town == end:
            end_time = time.time()
            current_mem,peak_mem = tracemalloc.get_traced_memory()
            tracemalloc.stop()

            return path, distance_traveled, end_time-start_time, current_mem,peak_mem
        
        #if town hasn't need visited then add town to set
        if current_town not in visited:
            visited.add(current_town)

            #iterates through the neighbors of the current town
            for neighbor in adjacency.get(current_town, []):
                if neighbor not in visited:

                    #estimated heuristic from the nieghbor to the end coordinate
                    heuristic = coordinate_length(coordinates[neighbor], coordinates[end])

                    #actual distance from current_town to neighbor
                    distance = coordinate_length(coordinates[current_town], coordinates[neighbor])

                    #add these values to the priority queue
                    heapq.heappush(priority_list, (heuristic, neighbor, path+ [neighbor], distance_traveled+distance))

    #end timer and memory usage
    end_time = time.time()
    current_mem,peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return None,0,end_time-start_time,current_mem,peak_mem


# A* Search
# adjacency: list of towns with adjacent towns
# coordinates: coordinates of the towns
# start: starting town picked by the user
# end: ending town picked by the user
def a_star_search(adjacency, coordinates, start,end):

    #start the timer and memory
    tracemalloc.start()
    start_time = time.time()

    #First value is initial value cost
    #second value is the starting city
    #third is the initial starting city
    #fourth is the distance traveled
    priority_list = [(0,start,[start],0)]

    #set to keep track of the visited cities
    visited = set()
    #stores the cost of the shortest path from start node to each visited node
    g_n_value = {start: 0}

    while priority_list:
        #f_value is the total cost of the current node: g+h
        #current_town is the current city being explored
        #path is the path taken to reach current_town
        #g_cost is the cost from the start node to current_town
        f_value, current_town, path,g_cost = heapq.heappop(priority_list)

        #if end town is reached then stop timer and memory
        if current_town == end:
            end_time = time.time()
            current_mem, peak_mem = tracemalloc.get_traced_memory()
            tracemalloc.stop()
            return path,g_cost, end_time-start_time,current_mem,peak_mem
        
        #if town hasn't been visited
        if current_town not in visited:
            #add town to visited set
            visited.add(current_town)


            for neighbor in adjacency.get(current_town, []):
                #calculate the distance from the current_town to the neighbor
                distance = coordinate_length(coordinates[current_town], coordinates[neighbor])
                #the sum of the cost to reach the town from the start
                tentative_g_cost = g_cost + distance

                #if neighbor hasn't been visited or the new cost is lower than previous cost
                if neighbor not in g_n_value or tentative_g_cost < g_n_value[neighbor]:
                    #update the cost for the visiting the nodes
                    g_n_value[neighbor] = tentative_g_cost
                    #estimated distance from the nighbor to the destination
                    h_cost = coordinate_length(coordinates[neighbor], coordinates[end])
                    #total cost of visiting nodes
                    f_value = tentative_g_cost + h_cost

                    #push onto queue with all the new information
                    heapq.heappush(priority_list, (f_value, neighbor,path + [neighbor], tentative_g_cost))

    #end timer and memory usage
    end_time = time.time()
    current_mem, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return None,0,end_time-start_time, current_mem, peak_mem





#Asks the user for whic town they'll start and end at

while True:

    starting_town = input("\nEnter the starting town: ")
    if(starting_town in coordinates):
        break
    else:
        print("Town doesnt exist, please try again")
        
while True:
    ending_town = input("Enter the town you'll end at: ")

    if ending_town in coordinates:
        break
    else:
        print("Town doesnt exist, please try again")

while True:

    
    graph_algorithm = input("\nWhich traversal would you like: \n"+
                                "A: Breadth-First Search\n" +
                                "B: Depth-First Search\n" +
                                "C: ID-DFS Search\n" +
                                "D: Best-First Search\n" +
                                "E: A* Search\n" +
                                "Enter: ")
    
    pick = graph_algorithm.upper()

    if(pick != "A" and pick != "B" and pick != "C" and pick != "D" and pick != "E"):
        print("Please choose a valid option")
    else:
        if pick == 'A':
            path, total_distance, time_taken, current_memory, peak_memory = breadth_first_search(adjacency, coordinates, starting_town, ending_town)
            print("Breadth-First Search")
            print(f"Shortest path from {starting_town} to {ending_town}: {' -> '.join(path)}")
            print(f"Time taken: {time_taken:.6f} seconds")
            print(f"Current memory usage: {current_memory / 1024:.2f} KB")
            print(f"Peak memory usage: {peak_memory / 1024:.2f} KB")
        elif pick == 'B':
            path, total_distance, time_taken, current_memory, peak_memory = depth_first_search(adjacency, coordinates, starting_town, ending_town)
            print("Depth-First Search")
            print(f"Shortest path from {starting_town} to {ending_town}: {' -> '.join(path)}")
            print(f"Time taken: {time_taken:.6f} seconds")
            print(f"Current memory usage: {current_memory / 1024:.2f} KB")
            print(f"Peak memory usage: {peak_memory / 1024:.2f} KB")
        elif pick == 'C':
            while True:
                max_depth_value = input("Please give max depth value: ")
                if max_depth_value.isdigit():
                    max_depth_value = int(max_depth_value)
                    break
                else:
                    print("Please enter a valid integer")

            path, total_distance, time_taken, current_memory, peak_memory = iddfs(adjacency, coordinates, starting_town, ending_town, max_depth_value)
            print("ID-DFS Search")
            print(f"Shortest path from {starting_town} to {ending_town}: {' -> '.join(path)}")
            print(f"Time taken: {time_taken:.6f} seconds")
            print(f"Current memory usage: {current_memory / 1024:.2f} KB")
            print(f"Peak memory usage: {peak_memory / 1024:.2f} KB")
        elif pick == 'D':
            path, total_distance, time_taken, current_memory, peak_memory = best_first_search(adjacency, coordinates, starting_town, ending_town)
            print("Best-First Search")
            print(f"Shortest path from {starting_town} to {ending_town}: {' -> '.join(path)}")
            print(f"Time taken: {time_taken:.6f} seconds")
            print(f"Current memory usage: {current_memory / 1024:.2f} KB")
            print(f"Peak memory usage: {peak_memory / 1024:.2f} KB")
        elif pick == 'E':
            path, total_distance, time_taken, current_memory, peak_memory = a_star_search(adjacency, coordinates, starting_town, ending_town)
            print("A* Search")
            print(f"Shortest path from {starting_town} to {ending_town}: {' -> '.join(path)}")
            print(f"Time taken: {time_taken:.6f} seconds")
            print(f"Current memory usage: {current_memory / 1024:.2f} KB")
            print(f"Peak memory usage: {peak_memory / 1024:.2f} KB")


