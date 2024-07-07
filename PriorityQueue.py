"""
Lucas Krüger, 222201036
Henry Freyschmidt, 222201945
Steven Zabel, 222200518
Paula Krasnovska, 222200392
Zana Salih Hama, 222200806
"""

import heapq

class PriorityQueue:
  """Priority Queue mit < ist besser"""

  def __init__(self):
    self.elements = []

  def empty(self):
    return len(self.elements) == 0

  def put(self, item, priority):
    heapq.heappush(self.elements, (priority, item))

  def get(self):
    return heapq.heappop(self.elements)[1]
