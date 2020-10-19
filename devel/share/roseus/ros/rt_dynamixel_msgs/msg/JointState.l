;; Auto-generated. Do not edit!


(when (boundp 'rt_dynamixel_msgs::JointState)
  (if (not (find-package "RT_DYNAMIXEL_MSGS"))
    (make-package "RT_DYNAMIXEL_MSGS"))
  (shadow 'JointState (find-package "RT_DYNAMIXEL_MSGS")))
(unless (find-package "RT_DYNAMIXEL_MSGS::JOINTSTATE")
  (make-package "RT_DYNAMIXEL_MSGS::JOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude JointState.msg.html


(intern "*UPDATED*" (find-package "RT_DYNAMIXEL_MSGS::JOINTSTATE"))
(shadow '*UPDATED* (find-package "RT_DYNAMIXEL_MSGS::JOINTSTATE"))
(defconstant rt_dynamixel_msgs::JointState::*UPDATED* 0)
(intern "*LOST*" (find-package "RT_DYNAMIXEL_MSGS::JOINTSTATE"))
(shadow '*LOST* (find-package "RT_DYNAMIXEL_MSGS::JOINTSTATE"))
(defconstant rt_dynamixel_msgs::JointState::*LOST* 128)
(defclass rt_dynamixel_msgs::JointState
  :super ros::object
  :slots (_id _updated _angle _velocity _current ))

(defmethod rt_dynamixel_msgs::JointState
  (:init
   (&key
    ((:id __id) (make-array 0 :initial-element 0 :element-type :char))
    ((:updated __updated) (make-array 0 :initial-element 0 :element-type :char))
    ((:angle __angle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:current __current) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id __id)
   (setq _updated __updated)
   (setq _angle __angle)
   (setq _velocity __velocity)
   (setq _current __current)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:updated
   (&optional __updated)
   (if __updated (setq _updated __updated)) _updated)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; uint8[] _id
    (* 1    (length _id)) 4
    ;; uint8[] _updated
    (* 1    (length _updated)) 4
    ;; float64[] _angle
    (* 8    (length _angle)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; float64[] _current
    (* 8    (length _current)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _id
     (write-long (length _id) s)
     (princ _id s)
     ;; uint8[] _updated
     (write-long (length _updated) s)
     (princ _updated s)
     ;; float64[] _angle
     (write-long (length _angle) s)
     (dotimes (i (length _angle))
       (sys::poke (elt _angle i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _current
     (write-long (length _current) s)
     (dotimes (i (length _current))
       (sys::poke (elt _current i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _id (make-array n :element-type :char))
     (replace _id buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _updated
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _updated (make-array n :element-type :char))
     (replace _updated buf :start2 ptr-) (incf ptr- n))
   ;; float64[] _angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angle i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _current
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _current i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get rt_dynamixel_msgs::JointState :md5sum-) "06ba1356a0bc81b57a392e7b27a8749e")
(setf (get rt_dynamixel_msgs::JointState :datatype-) "rt_dynamixel_msgs/JointState")
(setf (get rt_dynamixel_msgs::JointState :definition-)
      "uint8 UPDATED=0
uint8 LOST=128

uint8[] id
uint8[] updated # error
float64[] angle
float64[] velocity
float64[] current


")



(provide :rt_dynamixel_msgs/JointState "06ba1356a0bc81b57a392e7b27a8749e")


