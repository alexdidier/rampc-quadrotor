
(cl:in-package :asdf)

(defsystem "crazypkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorCommands" :depends-on ("_package_MotorCommands"))
    (:file "_package_MotorCommands" :depends-on ("_package"))
    (:file "SampleTimeParam" :depends-on ("_package_SampleTimeParam"))
    (:file "_package_SampleTimeParam" :depends-on ("_package"))
    (:file "ViconData" :depends-on ("_package_ViconData"))
    (:file "_package_ViconData" :depends-on ("_package"))
    (:file "ControllerOutputPackage" :depends-on ("_package_ControllerOutputPackage"))
    (:file "_package_ControllerOutputPackage" :depends-on ("_package"))
    (:file "ControllerParam" :depends-on ("_package_ControllerParam"))
    (:file "_package_ControllerParam" :depends-on ("_package"))
    (:file "PositionSetpoint" :depends-on ("_package_PositionSetpoint"))
    (:file "_package_PositionSetpoint" :depends-on ("_package"))
  ))