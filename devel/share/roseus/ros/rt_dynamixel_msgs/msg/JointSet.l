;; Auto-generated. Do not edit!


(when (boundp 'rt_dynamixel_msgs::JointSet)
  (if (not (find-package "RT_DYNAMIXEL_MSGS"))
    (make-package "RT_DYNAMIXEL_MSGS"))
  (shadow 'JointSet (find-package "RT_DYNAMIXEL_MSGS")))
(unless (find-package "RT_DYNAMIXEL_MSGS::JOINTSET")
  (make-package "RT_DYNAMIXEL_MSGS::JOINTSET"))

(in-package "ROS")
;;//! \htmlinclude JointSet.msg.html


(defclass rt_dynamixel_msgs::JointSet
  :super ros::object
  :slots (_id _angle ))

(defmethod rt_dynamixel_msgs::JointSet
  (:init
   (&key
    ((:id __id) (make-array 0 :initial-element 0 :element-type :char))
    ((:angle __angle) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id __id)
   (setq _angle __angle)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; uint8[] _id
    (* 1    (length _id)) 4
    ;; float64[] _angle
    (* 8    (length _angle)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _id
     (write-long (length _id) s)
     (princ _id s)
     ;; float64[] _angle
     (write-long (length _angle) s)
     (dotimes (i (length _angle))
       (sys::poke (elt _angle i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _id (make-array n :element-type :char))
     (replace _id buf :start2 ptr-) (incf ptr- n))
   ;; float64[] _angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angle i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get rt_dynamixel_msgs::JointSet :md5sum-) "b64b5c2f49125ccc72173634fc438fd8")
(setf (get rt_dynamixel_msgs::JointSet :datatype-) "rt_dynamixel_msgs/JointSet")
(setf (get rt_dynamixel_msgs::JointSet :definition-)
      "uint8[] id
float64[] angle


")



(provide :rt_dynamixel_msgs/JointSet "b64b5c2f49125ccc72173634fc438fd8")


