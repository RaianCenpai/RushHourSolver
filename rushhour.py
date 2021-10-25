import copy
import heapq

# This class extracts all relevant rush hour information from a list of strings and stores it. 
# It also has functions that can convert the board state to a list of strings, return if the goal state is reached,
# return heuristics based on the board state, make moves on the board, and generate possible moves.
class Board:
    

    # The constructor for Board iterates over every character in the list of strings and populates dictionaries coordinates,
    # lengths, and direction which store those values for each car specifically.
    # coordinates specifically stores all coordinates that the car takes up based on the fact that (0,0) is the grid in the top left corner.
    # lengths stores the length of the car.
    # direction stores the direction of the car as either h for horizontal or v for vertical.
    # The board size is also stored to support variable board sizes, however the car has to be in the 3 row from the top to work.
    def __init__(self, inputBoard):
        self.coordinates = {}
        self.lengths = {}
        self.direction = {}
        self.sizeX = len(inputBoard)
        self.sizeY = len(inputBoard[0])

        for i in range(len(inputBoard)):
            for j in range(len(inputBoard[i])):
                # Checks if a code letter already has a key in the coordinates dictionary and is a letter.
                if (inputBoard[i][j] not in self.coordinates.keys()) and (inputBoard[i][j].isalpha()):

                    length = 0

                    # Checks if incrementing would cause an out of bounds error in the horizontal direction.
                    if j+1 < len(inputBoard[i]):

                        # Increments the length as it continues to find characters horizontally.
                        while(inputBoard[i][j] == inputBoard[i][j+length+1]):
                            length += 1
                            if j+length+1 > self.sizeX-1:
                                break

                        # Assigns the coordinates, direction, and length to the the proper dictionaries
                        if length > 0:
                            self.coordinates[inputBoard[i][j]] = []
                            for adder in range(length+1):
                                self.coordinates[inputBoard[i][j]].append((j+adder,i))
                            self.direction[inputBoard[i][j]] = 'h'
                            self.lengths[inputBoard[i][j]] = length+1

                    # Checks if incrementing would cause an out of bounds error in the vertical direction.
                    if length == 0 and i+1 < len(inputBoard):

                        # Increments the length as it continues to find characters vertically.
                        while(inputBoard[i][j] == inputBoard[i+length+1][j]):
                            length += 1
                            if i+length+1 > self.sizeY-1:
                                break

                        # Assigns the coordinates, direction, and length to the the proper dictionaries
                        if length > 0:
                            self.coordinates[inputBoard[i][j]] = []
                            for adder in range(length+1):
                                self.coordinates[inputBoard[i][j]].append((j,i+adder))
                            self.direction[inputBoard[i][j]] = 'v'
                            self.lengths[inputBoard[i][j]] = length+1

    # Override for the == operator which checks that all the dictionaries are equal to each other.
    def __eq__(self, other):
        if self.coordinates == other.coordinates:
            if self.direction == other.direction:
                if self.lengths == other.lengths:
                    return(True)

        return(False)

    # getState populates a list of lists of characters with dashes then replaces the proper coordinates
    # with the proper characters, then returns it as a list of strings.
    def getState(self):
        boardOutput = []
        stringBoardOutput = []
        for i in range(self.sizeX):
            boardOutput.append([])
            for j in range(self.sizeY):
                boardOutput[i].append('-')

        for key in self.coordinates:
            for i in range(len(self.coordinates[key])):
                boardOutput[self.coordinates[key][i][1]][self.coordinates[key][i][0]] = key

        for i in range(len(boardOutput)):
            stringBoardOutput.append("".join(boardOutput[i]))

        return(stringBoardOutput)        


    # generateVerticalMoves takes a key and looks for the possible vertical moves by checking first if it is at the edge
    # of the board then if it is blocked by any cars by looking through the coordinates of every car that isn't itself.
    # It then returns a list of moves which is either [-1,1], [-1], [1], or [].
    # These correspond to the direction on the x-y grid of the board given that (0,0) starts in the top left.
    # This function runs under the assumption that the given key is a vertical car, which is checked for in the function that calls it.
    def generateVerticalMoves(self, key):

        verticalMoves = []
        keyFrontCoord = (self.coordinates[key][0][0], self.coordinates[key][0][1]-1)
        keyEndCoord = (self.coordinates[key][-1][0], self.coordinates[key][-1][1]+1)

        if keyFrontCoord[1] >= 0:
            blockedUp = 0
            for otherKeys in self.coordinates:
                if otherKeys != key:
                    for coordinate in self.coordinates[otherKeys]:
                        if coordinate == keyFrontCoord:
                            blockedUp = 1
            if blockedUp == 0:
                verticalMoves.append(-1)
        if keyEndCoord[1] <= self.sizeY-1:
            blockedDown = 0
            for otherKeys in self.coordinates:
                if otherKeys != key:
                    for coordinate in self.coordinates[otherKeys]:
                        if coordinate == keyEndCoord:
                            blockedDown = 1
            if blockedDown == 0:
                verticalMoves.append(1)

        return(verticalMoves)


    # generateHorizontalMoves takes a key and looks for the possible horizontal moves by checking first if it is at the edge
    # of the board then if it is blocked by any cars by looking through the coordinates of every car that isn't itself.
    # It then returns a list of moves which is either [-1,1], [-1], [1], or [].
    # These correspond to the direction on the x-y grid of the board given that (0,0) starts in the top left.
    # This function runs under the assumption that the given key is a horizontal car, which is checked for in the function that calls it.
    def generateHorizontalMoves(self, key):

        horizontalMoves = []
        keyFrontCoord = (self.coordinates[key][0][0]-1, self.coordinates[key][0][1])
        keyEndCoord = (self.coordinates[key][-1][0]+1, self.coordinates[key][-1][1])

        if keyFrontCoord[0] >= 0:
            blockedLeft = 0
            for otherKeys in self.coordinates:
                if otherKeys != key:
                    for coordinate in self.coordinates[otherKeys]:
                        if coordinate == keyFrontCoord:
                            blockedLeft = 1
            if blockedLeft == 0:
                horizontalMoves.append(-1)
        if keyEndCoord[0] <= self.sizeX-1:
            blockedRight = 0
            for otherKeys in self.coordinates:
                if otherKeys != key:
                    for coordinate in self.coordinates[otherKeys]:
                        if coordinate == keyEndCoord:
                            blockedRight = 1
            if blockedRight == 0:
                horizontalMoves.append(1)

        return(horizontalMoves)


    # generateAllMoves iterates on every car on the board and looks for every possible move.
    # The function looks at whether the car is vertical or horizontal and calls the proper function
    # then it adds the possible moves of that car to a dictionary with the key as the car code and returns it.
    def generateAllMoves(self):

        possibleMoves = {}
        for key in self.coordinates:
            if self.direction[key] == 'v':
                possibleMoves[key] = self.generateVerticalMoves(key)
            
            if self.direction[key] == 'h':
                possibleMoves[key] = self.generateHorizontalMoves(key)

        return(possibleMoves)


    # move performs a given move on a given car.
    # The function first checks if the car is vertical or horizontal and then it adds the given value to the proper
    # x or y value to make the car move vertically or horizontally.
    def move(self, key, value):
        
        if self.direction[key] == 'v':
            for i in range(len(self.coordinates[key])):
                self.coordinates[key][i] = (self.coordinates[key][i][0],self.coordinates[key][i][1]+value)
        elif self.direction[key] == 'h':
            for i in range(len(self.coordinates[key])):
                self.coordinates[key][i] = (self.coordinates[key][i][0]+value,self.coordinates[key][i][1])
    

    # getHeuristicBlockingCars returns the value of the heuristic based on every car that is blocking the main car 'X'
    # The function checks how many cars have coordinates that are on the third row from the top.
    # This function runs on the assumption that there is only vertical cars that can be blocking the main car.
    # However, a puzzle would not be solvable if there was a horizontal car blocking so I believe the assumption is just.
    def getHeuristicBlockingCars(self):

        blockingCars = 0
        possibleMoves = self.generateAllMoves()
        for key in self.coordinates:
            for coordinate in self.coordinates[key]:
                if coordinate[1] == 2 and key != 'X':
                    blockingCars += 1

        return(blockingCars)


    # getHeuristicBlockedBlockingCarsDistance is my self made heuristic which returns the number of cars blocking the main car
    # in addition to the number of those cars that are blocked themselves in addition to the distance the main car is from the goal.
    # So if there is 3 cars blocking the main car from the goal, but two of those cars themselves are blocked and the main car is 4
    # spots away from the goal, then 3 + 2 + 4 = 9.
    # My logic behind this heuristic is that if a car is blocking the main car it will need to be moved and if it can't be moved then
    # the car that is blocking it will also have to be moved. Therefore that is another move that has to be made. The distance is also
    # a reasonable additional number of moves until the goal state.
    def getHeuristicBlockedBlockingCarsDistance(self):

        blockingCars = self.getHeuristicBlockingCars()
        blockedBlockingCars = 0
        distance = (self.sizeX - 2) - self.coordinates['X'][0][0]
        possibleMoves = self.generateAllMoves()

        for key in self.coordinates:
            if key != 'X':
                if not possibleMoves[key]:
                    for coordinate in self.coordinates[key]:
                        if coordinate[1] == 2:
                            blockedBlockingCars += 1

        return(blockingCars + blockedBlockingCars + distance)


    def goalReached(self):

        return((self.sizeX - 2) == self.coordinates['X'][0][0])
        

# This class provides the necessary functions to traceback the path for the A* search.
# It stores the parent of the node and the state of the board as well as the g, h, and f values.
# g is the cost from the start to this node.
# h is the estimated cost from this node to the goal a.k.a. the heuristic.
# f is the combined value of these two.
class Node:
    
    def __init__(self, parent, state):
        self.parent = parent
        self.state = state

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return(self.state == other.state)

    def __lt__(self, other):
        return(self.f < other.f)


# This function is the main function to be called with the heuristic option and the input board that runs the A* search algorithm.
# It prints the best path to get to the goal from the starting position as well as the total moves and total states explored.
# It utilizes the heapq in order to extract the minimum value from the frontier.
def rushhour(heuristic, input):
    inputBoard = Board(input)
    inputHeuristic = heuristic
    frontier = []
    statesExplored = 0
    visited = []
    goalPath = []

    # Loads the first board state that is given.
    startNode = Node(None, inputBoard)
    heapq.heappush(frontier, startNode)

    # While loop continues while there are still unexplored states in the frontier.
    while(frontier):
        
        currentNode = heapq.heappop(frontier)
        visitedState = 0
        
        # This block of code ensures that states that are already visited are skipped.
        for node in visited:
            if currentNode.state == node.state:
                visitedState = 1
        if visitedState == 1:
            continue

        statesExplored += 1

        # This block of code populates the goalPath if the goal is reached by calling trace.
        if currentNode.state.goalReached():
            goalPath = trace(currentNode)
            break

        # Generates all the possible moves from the given board
        possibleMoves = currentNode.state.generateAllMoves()

        # Looks at every possible move by looking at each move in a given key.
        for key in possibleMoves:
            for value in possibleMoves[key]:

                # Makes a deepcopy of the board then makes the move that it is generating.
                newState = copy.deepcopy(currentNode.state)
                newState.move(key, value)

                # Creates a new child node of the current node with the new board state.
                childNode = Node(currentNode, newState)

                # Updates the g value.
                childNode.g = currentNode.g + 1

                # Updates the h value based on the chosen heuristic.
                if inputHeuristic == 0:
                    childNode.h = newState.getHeuristicBlockingCars()
                elif inputHeuristic == 1:
                    childNode.h = newState.getHeuristicBlockedBlockingCarsDistance()
                
                # Updates the f value.
                childNode.f = childNode.g + childNode.h
                
                # Pushes the new child to the minheap.
                heapq.heappush(frontier, childNode)

        visited.append(currentNode)

    # When the goalPath is found it prints out every row in every move in the goalPath as a string.
    for state in goalPath:
        for string in state.getState():
            print(string)

        print("")

    print("Total Moves:", len(goalPath))
    print("Total States Explored:", statesExplored)

# trace allows the A* program to traceback the path that it took to get to the goal state.
# Then it reverses the path so it is in the right order and returns.
def trace(endNode):
    path = []
    currentNode = endNode
    while(currentNode is not None):
        path.append(currentNode.state)
        currentNode = currentNode.parent

    path = path[::-1]
    return(path)