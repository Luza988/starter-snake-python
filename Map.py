import numpy as np

class Map:
  def __init__(self):
    self.size = -1
    self.map = None
    self.path = []
    self.last_Request = None

  def setup(self, size, request):
    self.map = np.zeros((size, size)).astype(int)
    self.size = size
    self.last_Request = request

  def set_wall(self, x, y):
    self.map[x][y] = -1


  """
  [
    {"x": 5, "y": 5},
    {"x": 9, "y": 0},
    {"x": 2, "y": 6}
  ]
  """
  def set_food(self, foodList):
    for food in foodList:
      self.map[food["x"]][food["y"]] = 1 

  def update_map(self, current_request):
    self.setup(current_request["board"]["height"], current_request)
    self.set_food(current_request["board"]["food"])
    self.set_snakes(current_request["board"]["snakes"])

  def set_snakes(self, snakes):
    for snake in snakes:
      bodys = snake["body"]
      for i in range(len(bodys)-1):
        body = bodys.pop(0)
        self.set_wall(body["x"], body["y"])
        if i == 0 and self.last_Request["you"]["head"] != body:  
          if snake["length"] + 2 < self.last_Request["you"]["length"]:
            self.map[body["x"]][body["y"]] = 1#self.zone(body["x"],body["y"], 1)    
          else:
            self.zone(body["x"], body["y"], -1)

  def zone(self, pos_x, pos_y, value):
    dw = [-1, +1, 0, 0]
    dh = [0, 0, +1, -1]
    for i in range(4):
      if 0 <= pos_x + dw[i] < self.size and 0 <= pos_y + dh[i] < self.size and self.map[pos_x + dw[i]][pos_y+dh[i]] != -1:
        self.map[pos_x + dw[i]][pos_y+dh[i]] = value

  def find_path(self, position:dict):
    to_visit = [(position["x"], position["y"], (position["x"], position["y"]))]
    visited_from = {}

    start = (position["x"], position["y"])
    found = None

    dw = [-1, +1, 0, 0]
    dh = [0, 0, +1, -1]
    #print(self.map)
    while to_visit:
      pos_x, pos_y, last_point = to_visit.pop(0)
      if self.map[pos_x][pos_y] == 1: # stop search when food found
        found = (pos_x, pos_y)
        visited_from[pos_x, pos_y] = last_point
        break
      for i in range(4):
        if 0 <= pos_x + dw[i] < self.size and 0 <= pos_y + dh[i] < self.size:
          visited_from[(pos_x, pos_y)] = last_point # save last step to current point
          #print(self.map[pos_x+dw[i]][pos_y+dh[i]])
          if visited_from.get((pos_x+dw[i], pos_y+dh[i]), []) != [] or self.map[pos_x+dw[i]][pos_y+dh[i]] == -1:
            continue
          to_visit.append((pos_x+dw[i], pos_y+ dh[i], (pos_x, pos_y)))

    self.path = []
    if found:
      while found != start:
        self.path.append(found)
        found = visited_from[found]
    else:
      for i in range(4):
        #print(dw[i], dh[i])
        if 0 <= pos_x + dw[i] < self.size and 0 <= pos_y + dh[i] < self.size and self.map[pos_x+dw[i]][pos_y+dh[i]] != -1:
          self.path = [(pos_x + dw[i], pos_y+dh[i])]
          break

    self.path = self.path.copy()[::-1]

  def next_step(self, position:dict):
    self.find_path(position)
    #print(self.path)
    ziel = self.path.pop(0)
    return self.direction(position, ziel)

  # Valid moves are "up", "down", "left", or "right"
  def direction(self, position: dict, ziel):
    px = position["x"]
    py = position["y"]
    zx, zy = ziel
    if px == zx:
      if py > zy:
        return "down"
      else:
        return "up"
    elif py == zy:
      if px > zx:
        return "left"
      else:
        return "right"
    else:
      return self.valid_move()

  def valid_move(self):
    snake_pos = self.last_Request["you"]["head"]
    pos_x = snake_pos["x"]
    pos_y = snake_pos["y"]
    dw = [-1, +1, 0, 0]
    dh = [0, 0, +1, -1]
    direction = ["left", "right", "up", "down"]
    for i in range(4):
      print(pos_x + dw[i],pos_y + dh[i]) 
      if 0 <= pos_x + dw[i] < self.size and 0 <= pos_y + dh[i] < self.size and self.map[pos_x+dw[i]][pos_y+dh[i]] != -1:
        return direction[i]