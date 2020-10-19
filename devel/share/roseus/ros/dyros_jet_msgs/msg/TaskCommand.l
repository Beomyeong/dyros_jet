;; Auto-generated. Do not edit!


(when (boundp 'dyros_jet_msgs::TaskCommand)
  (if (not (find-package "DYROS_JET_MSGS"))
    (make-package "DYROS_JET_MSGS"))
  (shadow 'TaskCommand (find-package "DYROS_JET_MSGS")))
(unless (find-package "DYROS_JET_MSGS::TASKCOMMAND")
  (make-package "DYROS_JET_MSGS::TASKCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude TaskCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*EE_LEFT_FOOT*" (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(shadow '*EE_LEFT_FOOT* (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(defconstant dyros_jet_msgs::TaskCommand::*EE_LEFT_FOOT* 0)
(intern "*EE_RIGHT_FOOT*" (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(shadow '*EE_RIGHT_FOOT* (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(defconstant dyros_jet_msgs::TaskCommand::*EE_RIGHT_FOOT* 1)
(intern "*EE_LEFT_HAND*" (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(shadow '*EE_LEFT_HAND* (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(defconstant dyros_jet_msgs::TaskCommand::*EE_LEFT_HAND* 2)
(intern "*EE_RIGHT_HAND*" (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(shadow '*EE_RIGHT_HAND* (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(defconstant dyros_jet_msgs::TaskCommand::*EE_RIGHT_HAND* 3)
(intern "*RELATIVE*" (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(shadow '*RELATIVE* (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(defconstant dyros_jet_msgs::TaskCommand::*RELATIVE* 0)
(intern "*ABSOLUTE*" (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(shadow '*ABSOLUTE* (find-package "DYROS_JET_MSGS::TASKCOMMAND"))
(defconstant dyros_jet_msgs::TaskCommand::*ABSOLUTE* 1)
(defclass dyros_jet_msgs::TaskCommand
  :super ros::object
  :slots (_header _end_effector _mode _pose _duration ))

(defmethod dyros_jet_msgs::TaskCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:end_effector __end_effector) (let (r) (dotimes (i 4) (push nil r)) r))
    ((:mode __mode) (make-array 4 :initial-element 0 :element-type :integer))
    ((:pose __pose) (let (r) (dotimes (i 4) (push (instance geometry_msgs::Pose :init) r)) r))
    ((:duration __duration) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _end_effector __end_effector)
   (setq _mode __mode)
   (setq _pose __pose)
   (setq _duration __duration)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:end_effector
   (&optional __end_effector)
   (if __end_effector (setq _end_effector __end_effector)) _end_effector)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool[4] _end_effector
    (* 1    4)
    ;; uint32[4] _mode
    (* 4    4)
    ;; geometry_msgs/Pose[4] _pose
    (apply #'+ (send-all _pose :serialization-length))
    ;; float64[4] _duration
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool[4] _end_effector
     (dotimes (i 4)
       (if (elt _end_effector i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; uint32[4] _mode
     (dotimes (i 4)
       (write-long (elt _mode i) s)
       )
     ;; geometry_msgs/Pose[4] _pose
     (dolist (elem _pose)
       (send elem :serialize s)
       )
     ;; float64[4] _duration
     (dotimes (i 4)
       (sys::poke (elt _duration i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool[4] _end_effector
   (dotimes (i (length _end_effector))
     (setf (elt _end_effector i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; uint32[4] _mode
   (dotimes (i (length _mode))
     (setf (elt _mode i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; geometry_msgs/Pose[4] _pose
   (dotimes (i 4)
     (send (elt _pose i) :deserialize buf ptr-) (incf ptr- (send (elt _pose i) :serialization-length))
     )
   ;; float64[4] _duration
   (dotimes (i (length _duration))
     (setf (elt _duration i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get dyros_jet_msgs::TaskCommand :md5sum-) "636def6d1efe8a9dd71ed012354704b8")
(setf (get dyros_jet_msgs::TaskCommand :datatype-) "dyros_jet_msgs/TaskCommand")
(setf (get dyros_jet_msgs::TaskCommand :definition-)
      "std_msgs/Header header

# end_effector = true -> use 
bool[4] end_effector
uint32 EE_LEFT_FOOT = 0
uint32 EE_RIGHT_FOOT = 1
uint32 EE_LEFT_HAND = 2
uint32 EE_RIGHT_HAND = 3

uint32[4] mode
uint32 RELATIVE=0
uint32 ABSOLUTE=1

geometry_msgs/Pose[4] pose

float64[4] duration

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :dyros_jet_msgs/TaskCommand "636def6d1efe8a9dd71ed012354704b8")


