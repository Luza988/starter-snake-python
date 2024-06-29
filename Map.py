import numpy as np
import heapq
import sys


class PriorityQueue:

  def __init__(self):
    self.elements = []

  def empty(self):
    return len(self.elements) == 0

  def put(self, item, priority):
    heapq.heappush(self.elements, (priority, item))

  def get(self):
    return heapq.heappop(self.elements)[1]


class Map:

  def __init__(self):
    self.size = -1
    self.map = None
    self.path = []
    self.last_Request = None
    self.goals = set()

  def setup(self, size, request):
    self.map = np.ones((size, size)).astype(int)
    self.size = size
    self.last_Request = request
    self.goals.clear()

  def set_wall(self, x, y):
    self.map[x][y] = -2000
    self.zone(x, y, -3)

  def set_food(self, foodList, snakes):
    for food in foodList:
      if self.last_Request["you"]["health"] < 20:  # need food to survive
        self.map[food["x"]][food["y"]] = 3
        self.goals.add((food["x"], food["y"]))

      else:  # add food depending on other snakes
        my_distance = distance(
          (food["x"], food["y"]),
          (self.last_Request["you"]["head"]["x"],
           self.last_Request["you"]["head"]["y"]))  # distance to food
        min_snake_distance = 9999
        min_length = 0
        for snake in snakes:
          if snake["id"] == self.last_Request["you"]["id"]:
            continue
          snake_distance = distance(
            (food["x"], food["y"]),
            (snake["head"]["x"],
             snake["head"]["y"]))  # distance of other snake to food
          if snake_distance < min_snake_distance or (  # snake closer than other snakes
              snake_distance ==
              min_snake_distance  # snake same distance as other snakes but longer
              and min_length < snake["length"]):
            min_snake_distance = snake_distance  # set new distance
            min_length = snake["length"]  # set new length

        if min_snake_distance > my_distance or (  # other snake further away
            min_snake_distance ==
            my_distance  # other snake has same distance but is shorter
            and min_length < self.last_Request["you"]["length"]):
          self.map[food["x"]][food["y"]] = 3
          self.goals.add((food["x"], food["y"]))

  def set_outer_rim(self):
    for i in range(self.size):
      for j in range(self.size):
        if j % (self.size - 1) == 0 or i % (self.size - 1) == 0:
          self.map[i][j] += -4

  def update_map(self, current_request):
    self.setup(current_request["board"]["height"], current_request)
    self.set_outer_rim()
    self.set_snakes(current_request["board"]["snakes"],
                    current_request["board"]["food"])
    self.set_food(current_request["board"]["food"],
                  current_request["board"]["snakes"])

  def set_snakes(self, snakes, foods):
    for snake in snakes:
      bodys = snake["body"]
      # body other than tail
      for i in range(snake["length"] - 1):
        body = bodys[i]
        self.set_wall(body["x"], body["y"])
        if i == 0 and self.last_Request["you"]["head"] != body:
          if snake["length"] + 2 < self.last_Request["you"][
              "length"] and self.last_Request["you"]["health"] > 50:
            self.zone(body["x"], body["y"], 3)
          else:
            self.zone(body["x"], body["y"], -3)
      # tail
      if self.last_Request["you"]["id"] != snake["id"]:  # never add own tail
        continue
      dw = [-1, +1, 0, 0]
      dh = [0, 0, +1, -1]
      tail = False
      pos_x = snake["body"][0]["x"]  # x of head
      pos_y = snake["body"][0]["y"]  # y of head
      for i in range(4):
        if 0 <= pos_x + dw[i] < self.size and 0 <= pos_y + dh[
            i] < self.size:  # if cell next to head in bounds
          for food in foods:
            if food["x"] == pos_x + dw[i] and food[
                "y"] == pos_y + dh[i]:  # if any food in cell next to head
              tail = True  # need to add tail
              break
        if tail:
          break
      if tail:
        self.set_wall(snake["body"][snake["length"] - 1]["x"],
                      snake["body"][snake["length"] - 1]["y"])  # add tail

  def zone(self, pos_x, pos_y, value):
    dw = [-1, +1, 0, 0]
    dh = [0, 0, +1, -1]
    for i in range(4):
      if 0 <= pos_x + dw[i] < self.size and 0 <= pos_y + dh[
          i] < self.size and self.map[pos_x + dw[i]][pos_y + dh[i]] > -999:
        self.map[pos_x + dw[i]][pos_y + dh[i]] += value
        if value == 3:
          self.goals.add((pos_x + dw[i], pos_y + dh[i]))
        elif (pos_x + dw[i], pos_y + dh[i]) in self.goals:
          self.goals.discard((pos_x + dw[i], pos_y + dh[i]))

  def next_step(self, position: dict):
    copy = self.map.copy()
    print(np.rot90(copy, 1, (0, 1)))
    print("goal: ", self.goals)

    # search goal
    #print(position)
    path1 = self.astar((position["x"], position["y"]))
    ziel1 = path1.pop()
    ziel = ziel1

    # alternative goal
    if len(self.goals)-1 > 0:
      self.goals.discard(ziel)
      path2 = self.astar((position["x"], position["y"]))
      ziel2 = path2.pop()

      # compare goals and return better one
      if (len(path1) == len(path2)):
        if self.map[ziel1[0]][ziel1[1]] < self.map[ziel2[0]][ziel2[1]]:
          ziel = ziel2
    return self.direction(position, ziel)

  # Valid moves are "up", "down", "left", or "right"
  def direction(self, position: dict, ziel):
    if ziel is not None:
      px = position["x"]
      py = position["y"]
      zx, zy = ziel
      if px == zx:
        if py > zy:
          return "down"
        elif py != zy:
          return "up"
      elif py == zy:
        if px > zx:
          return "left"
        elif px != zx:
          return "right"

    return self.valid_move()

  def valid_move(self):
    snake_pos = self.last_Request["you"]["head"]
    pos_x = snake_pos["x"]
    pos_y = snake_pos["y"]
    dw = [-1, +1, 0, 0]
    dh = [0, 0, +1, -1]
    direction = ["left", "right", "up", "down"]
    maxi = 0
    max = -2000
    for i in range(4):
      if 0 <= pos_x + dw[i] < self.size and 0 <= pos_y + dh[  # if in bounds
          i] < self.size and self.map[pos_x + dw[i]][
            pos_y + dh[i]] > max:  # if cell has higher value than other cells
        max = self.map[pos_x + dw[i]][pos_y + dh[i]]
        maxi = i
    #print(self.last_Request)
    return direction[maxi]

  # A*
  def heuristic(self, position):
    min = sys.maxsize
    for goal in self.goals:
      multp = 1
      dx, dy = 0, 0
      if abs(goal[0] - position[0]) >= abs(goal[1] - position[1]):
        if goal[0] > position[0]:
          dx = 1
        elif goal[0] < position[0]:
          dx = -1
      else:
        if goal[1] > position[1]:
          dy = 1
        elif goal[1] < position[1]:
          dy = -1

      value = self.map[position[0] + dx][position[1] + dy]
      if value > 0:
        multp = value / 10
      elif value != 0:
        multp = abs(value)

      dis = multp * distance(position, goal)
      if dis < min:
        min = dis

    return min

  def astar(self, start):
    found = False
    unvisited_nodes = PriorityQueue()
    unvisited_nodes.put(start, 0)

    visited_nodes = {}
    visited_nodes[start] = None

    cost_so_far = {}
    cost_so_far[start] = 0

    while not unvisited_nodes.empty():
      current = unvisited_nodes.get()
      if current in self.goals:
        print("found goal", (current[0], current[1]))
        found = True
        break

      dw = [-1, +1, 0, 0]
      dh = [0, 0, +1, -1]
      for i in range(4):
        if 0 <= current[0] + dw[i] < self.size and 0 <= current[1] + dh[
            i] < self.size and self.map[current[0] + dw[i]][current[1] +
                                                            dh[i]] > -999:
          new_cost = cost_so_far[current] + 1
          if ((current[0] + dw[i], current[1] + dh[i])
              not in cost_so_far.keys()) or (new_cost < cost_so_far[(
                current[0] + dw[i], current[1] + dh[i])]):
            cost_so_far[(current[0] + dw[i], current[1] + dh[i])] = new_cost
            priorität = new_cost + self.heuristic(
              (current[0] + dw[i], current[1] + dh[i]))
            unvisited_nodes.put((current[0] + dw[i], current[1] + dh[i]),
                                priorität)
            visited_nodes[(current[0] + dw[i], current[1] + dh[i])] = current

    if found:
      path = self.make_path(visited_nodes, start, current)
    else:
      path = [start]
    return path

  def make_path(self, came_from, start, goal):
    path = [goal]
    if start == goal:
      return [start]
    nex = came_from[goal]
    while nex != start:
      path.append(nex)
      nex = came_from[nex]
    return path


# Unabhängige Funktionen


def distance(position1, position2):
  px, py = position1
  zx, zy = position2
  return abs(px - zx) + abs(py - zy)


#DeadEndFinder


def longest_path_finder(self, position, memory):
  x, y = position
  if memory[x][y] > -1:
    return memory[x][y]
  moves = [0, 0, 0, 0]  # 0: right, 1: left, 2: up, 3: down
  if x < self.size - 1:
    if self.map[x + 1][y] > -1000:  # is snake there?
      moves[0] = 1 + longest_path_finder(self, (x + 1, y))
  if x > 0:
    if self.map[x - 1][y] > -1000:
      moves[1] = 1 + longest_path_finder(self, (x - 1, y))
  if y < self.size - 1:
    if self.map[x][y + 1] > -1000:
      moves[2] = 1 + longest_path_finder(self, (x, y + 1))
  if y > 0:
    if self.map[x][y - 1] > -1000:
      moves[3] = 1 + longest_path_finder(self, (x, y - 1))
  memory[x][y] = max(moves)
  return max(moves)


#def dead_end(map, position):
