import random
import matplotlib.pyplot as plt
import numpy as np
import time
import math
import heapq


t_rate = 0.5 #target arrival rate per second
T = 5 #how many seconds each node will sleep
num_sensors = 150

class Sensor:
	def __init__(self, time):
		self.x = random.randint(0, 1000)
		self.y = random.randint(0, 1000)
		self.energy = 1000000000 #1 joule represented in nanojoules
		self.buf = []
		self.state = 'Active'
		self.time = time #how much longer the node will stay in the current state
		self.lastTargetTime = -1 #tick count of the last time this sensor recording a target
		self.color = 'green'
		self.neighbors = {} #nodes within range
		self.next_node = None
		
		def __lt__(self): #prevents an error when nodes are compared to eachother
			return False
		
	def send(self):
		if self.next_node is not None:
			if (self.next_node.state == 'Active' or self.next_node.state == 'Gateway'): #checking if next_node is asleep
				d = nodeDistance(self, self.next_node)
				self.energy = self.energy - ((25 + 0.05*(d**2))*10000)
				if self.next_node.state == 'Gateway': #if next_node is the gateway, set received time for packet
					self.buf[0].gateway_time = counter
				if (self.next_node.state == 'Active'): #if next_node is a sensor, subtract energy from it
					self.next_node.energy = self.next_node.energy - (25 * 10000)
				self.buf[0].energy += (((25 + 0.05*(d**2))*10000)+(25 * 10000))
				self.next_node.buf.append(self.buf[0]) #append message to destination node's buffer
				self.buf.pop(0) #remove message from self buffer

			#calculate energy loss and adjust for this node and receiving node
			
		
	def checkForTarget(self):
		for target in allTargets:
			if (nodeDistance(self, target)) <= 50: #checks if target distance is within range
				return True
				
	def stayAwake(self):
		if (self.checkForTarget()) or (len(self.buf) != 0): #checks if target is in range, or if buffer has packets
			return True
	
class Gateway:
	def __init__(self): #initializes gateway node
		self.x = random.randint(0, 1000)
		self.y = random.randint(0, 1000)
		self.neighbors = {}
		self.state = 'Gateway'
		self.buf = []
			
class Target:
	def __init__(self, seed, speed): #creates random target with random movement within bounds
		if seed == 0:
			self.x = 0 #spawns on left border
			self.y = random.randint(250, 750)
			self.dx = speed/100
			self.dy = random.randint((0-speed), speed) / 100
		if seed == 1:
			self.x = 1000 #spawns on right border
			self.y = random.randint(250, 750)
			self.dx = (0-speed)/100
			self.dy = random.randint((0-speed), speed) / 100
		if seed == 2:
			self.x = random.randint(250, 750)
			self.y = 1000 #spawns on top border
			self.dx = random.randint((0-speed), speed) / 100
			self.dy = (0-speed)/100
		if seed == 3:
			self.x = random.randint(250, 750)
			self.y = 0 #spawns on bottom border
			self.dx = random.randint((0-speed), speed) / 100
			self.dy = speed/100
			
			
		self.state = 'Target'
		
class Packet:
	def __init__(self, time):
		self.time = time #time when packet was recorded
		self.gateway_time = 0 #time when packet reaches the gateway node
		self.energy = 0
		
def nodeDistance(n1, n2): #calculates distance between 2 nodes
	return (math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2))

def dijkstra(allNodes):
    distances = {node: math.inf for node in allNodes} #initializes all distances to infinite
    distances[gate] = 0 #the distance from the gateway_node to itself is 0

    next_nodes = {node: None for node in allNodes} #dictionary to store each nodes next node in path
    pqueue = [(0, gate)] #gateway_node is the start of the pqueue

    while pqueue:
        distance, node = heapq.heappop(pqueue) #returns current nodes distance from the pqueue
        for x, y in node.neighbors.items(): #iterates through neighbors and checks distance
            if y <= 140:
                new_distance = distance + y 
                if new_distance < distances[x]: #this node is closer
                    distances[x] = new_distance
                    next_nodes[x] = node  #updates next node
                    heapq.heappush(pqueue, (new_distance, x))
    return next_nodes





#creating sensors
nodes = []
for i in range(num_sensors):
	nodes.append(Sensor(random.randint(1,1000))) #initial active state time is randomized between 1-10
	
#creating gateway
gate = Gateway()


#stores the neighbors of each node within 140m
for node in nodes:
    for j in nodes + [gate]:
        if node != j:
            distance = nodeDistance(node, j)
            if distance <= 140:
                node.neighbors[j] = distance
                j.neighbors[node] = distance #connects neighbors both ways

#uses dijkstra() to store the next_node value for each node
next_nodes = dijkstra(nodes + [gate])

#sets next_node value for each node from the list of next_nodes
for node, next_node in next_nodes.items():
    if next_node is not None: #next_node has been set
        node.next_node = next_node

#creating list to store targets
allTargets = []
	

fig, ax = plt.subplots()
#initializing and populating scatterplot
scatter_sensors = ax.scatter([node.x for node in nodes], [node.y for node in nodes], c=[node.color for node in nodes])
scatter_targets = ax.scatter([node.x for node in allTargets], [node.y for node in allTargets], color='red')
scatter_gateway = ax.scatter([gate.x], [gate.y], color='yellow')
ax.set_xlim(0, 1000)
ax.set_ylim(0, 1000)


for node in nodes: #drawing lines between connected nodes
	if node.next_node is not None:
		plt.plot([node.x, node.next_node.x], [node.y, node.next_node.y], color ='black', linestyle = '--', linewidth=1)


T = T*100 #accounting for each tick being 1/100th of a second
spawn_rate = (100/t_rate) #create a new target every t_rate ticks
lastTarget = -1 #time when last target was created

counter = 0 #each tick is 10 ms
seenTarget = False #flag that turns true after first target is detected
firstTargetTime = -1 #stores the time of when the first target is spotted

timer = ax.text(0.5, -0.1, f"", ha='center', va='center', fontsize=12, color='red')

while(True):
	for node in nodes:
		if node.energy <= 0: #exit simulation when the first node dies
			print("Time Until First Node Dies: " + str(counter/100) + " seconds.\n")
			print("Network Throughput: " + str(10 * (len(gate.buf) / (counter/100))) + " KBits per second.\n")
			average = 0
			for i in gate.buf:
				average += (i.gateway_time - i.time)
			average = (average/100) / len(gate.buf) #prints average packet delay
			print("Average Delay Per Packet: " + str(average) + " seconds.\n")
			print("First Target Detected At: " + str(firstTargetTime) + " seconds.\n")
			quit()
		
		if(counter - lastTarget >= spawn_rate) or (lastTarget == -1): #create random target
			seed = random.randint(0, 3)
			allTargets.append(Target(seed, 5))
			lastTarget = counter #stores time of last target creation
		
		if node.time == 0: #node needs to switch states when node.time is 0
			if node.state == 'Sleep':
				node.state = 'Active'
				node.color = 'green'
				node.time = 5 #probes for 50 msecs
			elif node.state == 'Active':
				node.state = 'Sleep'
				node.color = 'blue'
				node.time = T #sleeps for T time
			scatter_sensors.set_facecolors([node.color for node in nodes]) #adjust colors in graph
			
		if (node.state == 'Active') and node.next_node is not None:
			if node.stayAwake(): #there are either messages to send or targets to track
				node.time = 200 #node stays active for 2 seconds
				if len(node.buf) > 0: #buffer has messages to send
					node.send()
				if node.checkForTarget(): #target is within 50 meters
					if(seenTarget == False): #first target detected
						firstTargetTime = counter/100
						seenTarget = True
					#if its been over 1 second since recording data, or if this sensor hasn't recorded data yet
					if ((counter - node.lastTargetTime) > 100) or (node.lastTargetTime == -1): 
						node.buf.append(Packet(counter))
						node.lastTargetTime = counter #reset the time of last recording
						node.energy = node.energy - 25000 #sensing energy dissipation for 1Kbit
				
		node.time -= 1 #decrements the lifetime of the nodes current state
		

		
	for target in allTargets: #deletes target if out of bounds, adjusts position otherwise
		if target.x > 1100 or target.x < -100 or target.y > 1100 or target.y < -100: #out of bounds
			allTargets.remove(target)
		else:
			target.x = target.x + target.dx #adjusts target position by their dx and dy values
			target.y = target.y + target.dy

	scatter_targets.set_offsets(np.column_stack(([target.x for target in allTargets], [target.y for target in allTargets])))
	#updates target positions on scatterplot

	timer.set_text(f"Time: {counter//100} seconds") #shows time in scatterplot
	
	counter += 1
	
	plt.pause(0.0001)

plt.show()
		




