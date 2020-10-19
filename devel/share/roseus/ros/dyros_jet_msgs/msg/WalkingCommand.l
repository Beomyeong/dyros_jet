;; Auto-generated. Do not edit!


(when (boundp 'dyros_jet_msgs::WalkingCommand)
  (if (not (find-package "DYROS_JET_MSGS"))
    (make-package "DYROS_JET_MSGS"))
  (shadow 'WalkingCommand (find-package "DYROS_JET_MSGS")))
(unless (find-package "DYROS_JET_MSGS::WALKINGCOMMAND")
  (make-package "DYROS_JET_MSGS::WALKINGCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude WalkingCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*NONE*" (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(shadow '*NONE* (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(defconstant dyros_jet_msgs::WalkingCommand::*NONE* 0)
(intern "*STATIC_WALKING*" (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(shadow '*STATIC_WALKING* (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(defconstant dyros_jet_msgs::WalkingCommand::*STATIC_WALKING* 1)
(intern "*HIP_COMPENSTOR*" (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(shadow '*HIP_COMPENSTOR* (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(defconstant dyros_jet_msgs::WalkingCommand::*HIP_COMPENSTOR* 0)
(intern "*EXTERNAL_ENCODER*" (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(shadow '*EXTERNAL_ENCODER* (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(defconstant dyros_jet_msgs::WalkingCommand::*EXTERNAL_ENCODER* 1)
(intern "*IK*" (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(shadow '*IK* (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(defconstant dyros_jet_msgs::WalkingCommand::*IK* 0)
(intern "*JACOBIAN*" (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(shadow '*JACOBIAN* (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(defconstant dyros_jet_msgs::WalkingCommand::*JACOBIAN* 1)
(intern "*QP*" (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(shadow '*QP* (find-package "DYROS_JET_MSGS::WALKINGCOMMAND"))
(defconstant dyros_jet_msgs::WalkingCommand::*QP* 2)
(defclass dyros_jet_msgs::WalkingCommand
  :super ros::object
  :slots (_header _walk_mode _compensator_mode _ik_mode _heel_toe _first_foot_step _x _y _z _height _theta _step_length_x _step_length_y _walking_pattern ))

(defmethod dyros_jet_msgs::WalkingCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:walk_mode __walk_mode) 0)
    ((:compensator_mode __compensator_mode) (let (r) (dotimes (i 2) (push nil r)) r))
    ((:ik_mode __ik_mode) 0)
    ((:heel_toe __heel_toe) nil)
    ((:first_foot_step __first_foot_step) nil)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:height __height) 0.0)
    ((:theta __theta) 0.0)
    ((:step_length_x __step_length_x) 0.0)
    ((:step_length_y __step_length_y) 0.0)
    ((:walking_pattern __walking_pattern) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _walk_mode (round __walk_mode))
   (setq _compensator_mode __compensator_mode)
   (setq _ik_mode (round __ik_mode))
   (setq _heel_toe __heel_toe)
   (setq _first_foot_step __first_foot_step)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _height (float __height))
   (setq _theta (float __theta))
   (setq _step_length_x (float __step_length_x))
   (setq _step_length_y (float __step_length_y))
   (setq _walking_pattern __walking_pattern)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:walk_mode
   (&optional __walk_mode)
   (if __walk_mode (setq _walk_mode __walk_mode)) _walk_mode)
  (:compensator_mode
   (&optional __compensator_mode)
   (if __compensator_mode (setq _compensator_mode __compensator_mode)) _compensator_mode)
  (:ik_mode
   (&optional __ik_mode)
   (if __ik_mode (setq _ik_mode __ik_mode)) _ik_mode)
  (:heel_toe
   (&optional __heel_toe)
   (if __heel_toe (setq _heel_toe __heel_toe)) _heel_toe)
  (:first_foot_step
   (&optional __first_foot_step)
   (if __first_foot_step (setq _first_foot_step __first_foot_step)) _first_foot_step)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:step_length_x
   (&optional __step_length_x)
   (if __step_length_x (setq _step_length_x __step_length_x)) _step_length_x)
  (:step_length_y
   (&optional __step_length_y)
   (if __step_length_y (setq _step_length_y __step_length_y)) _step_length_y)
  (:walking_pattern
   (&optional __walking_pattern)
   (if __walking_pattern (setq _walking_pattern __walking_pattern)) _walking_pattern)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _walk_mode
    4
    ;; bool[2] _compensator_mode
    (* 1    2)
    ;; uint32 _ik_mode
    4
    ;; bool _heel_toe
    1
    ;; bool _first_foot_step
    1
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _height
    4
    ;; float32 _theta
    4
    ;; float32 _step_length_x
    4
    ;; float32 _step_length_y
    4
    ;; bool _walking_pattern
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _walk_mode
       (write-long _walk_mode s)
     ;; bool[2] _compensator_mode
     (dotimes (i 2)
       (if (elt _compensator_mode i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; uint32 _ik_mode
       (write-long _ik_mode s)
     ;; bool _heel_toe
       (if _heel_toe (write-byte -1 s) (write-byte 0 s))
     ;; bool _first_foot_step
       (if _first_foot_step (write-byte -1 s) (write-byte 0 s))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _step_length_x
       (sys::poke _step_length_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _step_length_y
       (sys::poke _step_length_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _walking_pattern
       (if _walking_pattern (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _walk_mode
     (setq _walk_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool[2] _compensator_mode
   (dotimes (i (length _compensator_mode))
     (setf (elt _compensator_mode i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; uint32 _ik_mode
     (setq _ik_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _heel_toe
     (setq _heel_toe (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _first_foot_step
     (setq _first_foot_step (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _step_length_x
     (setq _step_length_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _step_length_y
     (setq _step_length_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _walking_pattern
     (setq _walking_pattern (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dyros_jet_msgs::WalkingCommand :md5sum-) "7c98f46b805c25b4897e6489a2543122")
(setf (get dyros_jet_msgs::WalkingCommand :datatype-) "dyros_jet_msgs/WalkingCommand")
(setf (get dyros_jet_msgs::WalkingCommand :definition-)
      "std_msgs/Header header

uint32 walk_mode
uint32 NONE = 0
uint32 STATIC_WALKING = 1

bool[2] compensator_mode
uint32 HIP_COMPENSTOR = 0
uint32 EXTERNAL_ENCODER = 1

uint32 ik_mode
uint32 IK=0
uint32 JACOBIAN = 1
uint32 QP = 2

bool heel_toe

bool first_foot_step

float32 x
float32 y
float32 z
float32 height
float32 theta
float32 step_length_x
float32 step_length_y

bool walking_pattern

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

")



(provide :dyros_jet_msgs/WalkingCommand "7c98f46b805c25b4897e6489a2543122")


