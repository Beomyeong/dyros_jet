;; Auto-generated. Do not edit!


(when (boundp 'dyros_jet_msgs::JointState)
  (if (not (find-package "DYROS_JET_MSGS"))
    (make-package "DYROS_JET_MSGS"))
  (shadow 'JointState (find-package "DYROS_JET_MSGS")))
(unless (find-package "DYROS_JET_MSGS::JOINTSTATE")
  (make-package "DYROS_JET_MSGS::JOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude JointState.msg.html


(intern "*INPUT_VOLTAGE_ERROR*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*INPUT_VOLTAGE_ERROR* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*INPUT_VOLTAGE_ERROR* 1)
(intern "*MOTOR_HALL_SENSOR_ERROR*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*MOTOR_HALL_SENSOR_ERROR* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*MOTOR_HALL_SENSOR_ERROR* 2)
(intern "*OVERHEAT_ERROR*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*OVERHEAT_ERROR* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*OVERHEAT_ERROR* 4)
(intern "*MOTOR_ENCODER_ERROR*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*MOTOR_ENCODER_ERROR* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*MOTOR_ENCODER_ERROR* 8)
(intern "*ELECTRONICAL_SHOCK_ERROR*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*ELECTRONICAL_SHOCK_ERROR* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*ELECTRONICAL_SHOCK_ERROR* 16)
(intern "*OVERLOAD_ERROR*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*OVERLOAD_ERROR* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*OVERLOAD_ERROR* 32)
(intern "*RESERVED_ERROR*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*RESERVED_ERROR* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*RESERVED_ERROR* 64)
(intern "*NOT_UPDATED*" (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(shadow '*NOT_UPDATED* (find-package "DYROS_JET_MSGS::JOINTSTATE"))
(defconstant dyros_jet_msgs::JointState::*NOT_UPDATED* 128)
(defclass dyros_jet_msgs::JointState
  :super ros::object
  :slots (_name _angle _velocity _current _error ))

(defmethod dyros_jet_msgs::JointState
  (:init
   (&key
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:angle __angle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:current __current) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:error __error) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _name __name)
   (setq _angle __angle)
   (setq _velocity __velocity)
   (setq _current __current)
   (setq _error __error)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; float64[] _angle
    (* 8    (length _angle)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; float64[] _current
    (* 8    (length _current)) 4
    ;; uint32[] _error
    (* 4    (length _error)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
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
     ;; uint32[] _error
     (write-long (length _error) s)
     (dotimes (i (length _error))
       (write-long (elt _error i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
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
   ;; uint32[] _error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _error (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _error i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get dyros_jet_msgs::JointState :md5sum-) "44f97ce7a113b09b2cda2af039ff08e1")
(setf (get dyros_jet_msgs::JointState :datatype-) "dyros_jet_msgs/JointState")
(setf (get dyros_jet_msgs::JointState :definition-)
      "
# DXL Error
uint32 INPUT_VOLTAGE_ERROR	=1
uint32 MOTOR_HALL_SENSOR_ERROR	=2
uint32 OVERHEAT_ERROR		=4
uint32 MOTOR_ENCODER_ERROR	=8
uint32 ELECTRONICAL_SHOCK_ERROR	=16
uint32 OVERLOAD_ERROR		=32
uint32 RESERVED_ERROR		=64
uint32 NOT_UPDATED		=128


string[] name
float64[] angle
float64[] velocity
float64[] current

# bit 0000 0000 0000 0000 0000 0001 0000 0000 = Not updated
# bit 0000 0000 0000 0000 0000 0000 0000 0100 = Not updated, Overheat error
uint32[] error


")



(provide :dyros_jet_msgs/JointState "44f97ce7a113b09b2cda2af039ff08e1")


