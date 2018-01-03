/** \file
 * \brief Definition of the Subject class.
 */

#ifndef OBJECT_DETECTION_SUBJECT_H
#define OBJECT_DETECTION_SUBJECT_H

// std.
#include <vector>

// Headers of this project.
#include "observer.h"

// TODO: Allow observers to unregister.


namespace object_detection {

/** \brief Subject according to the OBSERVER pattern. */
class Subject {
public:
  /** \brief Adds an Observer to the internal vector of Observers. */
  void addObserver(Observer* observer);

  /** \brief Calls the update method of each registered Observer. */
  void notify() const;

private:
  std::vector<Observer*> observers_;
};

inline void Subject::addObserver(Observer* observer)
{
  if (observer)
    observers_.push_back(observer);
}

inline void Subject::notify() const
{
  for (auto observer : observers_)
    observer->update();
}

} // object_detection

#endif  // OBJECT_DETECTION_SUBJECT_H
