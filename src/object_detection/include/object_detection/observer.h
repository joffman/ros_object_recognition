/** \file
 * \brief Definition of the Observer class.
 */

#ifndef OBJECT_DETECTION_OBSERVER_H
#define OBJECT_DETECTION_OBSERVER_H

// TODO: Unregister from subject in destructor.


namespace object_detection {

/** \brief Observer according to the OBSERVER pattern. */
class Observer {
public:
  virtual ~Observer() {}

  /** \brief Updates the view on the Subject that the Observer presents. */
  virtual void update() = 0;
};

} // object_detection

#endif  // OBJECT_DETECTION_OBSERVER_H
