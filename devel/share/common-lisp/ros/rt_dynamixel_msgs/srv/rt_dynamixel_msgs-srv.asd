
(cl:in-package :asdf)

(defsystem "rt_dynamixel_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ModeSetting" :depends-on ("_package_ModeSetting"))
    (:file "_package_ModeSetting" :depends-on ("_package"))
    (:file "ModeSetting (copy)" :depends-on ("_package_ModeSetting (copy)"))
    (:file "_package_ModeSetting (copy)" :depends-on ("_package"))
    (:file "MotorSetting" :depends-on ("_package_MotorSetting"))
    (:file "_package_MotorSetting" :depends-on ("_package"))
    (:file "Status" :depends-on ("_package_Status"))
    (:file "_package_Status" :depends-on ("_package"))
  ))