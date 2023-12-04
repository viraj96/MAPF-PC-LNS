#pragma once

#include "constrainttable.hpp"

using Interval = tuple<int, int, bool>;  // [t_min, t_max), has collision

class ReservationTable {
 private:
  int goalLocation_;

  // Safe Interval Table (SIT)
  using SIT = vector<list<Interval>>;
  SIT safeIntervalTable_;  // location -> [t_min, t_max), num_of_collisions

  void insert2SIT(int location, int tMin, int tMax);
  void insertSoftConstraint2SIT(int location, int tMin, int tMax);

  // update SIT at the given location
  void updateSIT(int location);

  int getEarliestArrivalTime(int fromLocation, int toLocation, int lowerBound,
                             int upperBound) const;
  int getEarliestNoCollisionArrivalTime(int fromLocation, int toLocation,
                                        const Interval& interval,
                                        int lowerBound, int upperBound) const;

 public:
  const ConstraintTable& constraintTable;

  ReservationTable(const ConstraintTable& constraintTable)
      : goalLocation_(constraintTable.goalLocation),
        safeIntervalTable_(constraintTable.mapSize),
        constraintTable(constraintTable) {}

  list<tuple<int, int, int, bool, bool>> getSafeIntervals(int fromLocation,
                                                          int toLocation,
                                                          int lowerBound,
                                                          int upperBound);
  Interval getFirstSafeInterval(size_t location, int startTime);
  bool findSafeInterval(Interval& interval, size_t location, int tMin);
};
