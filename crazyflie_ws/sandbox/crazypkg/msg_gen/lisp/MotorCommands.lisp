; Auto-generated. Do not edit!


(cl:in-package crazypkg-msg)


;//! \htmlinclude MotorCommands.msg.html

(cl:defclass <MotorCommands> (roslisp-msg-protocol:ros-message)
  ((cmd1
    :reader cmd1
    :initarg :cmd1
    :type cl:float
    :initform 0.0)
   (cmd2
    :reader cmd2
    :initarg :cmd2
    :type cl:float
    :initform 0.0)
   (cmd3
    :reader cmd3
    :initarg :cmd3
    :type cl:float
    :initform 0.0)
   (cmd4
    :reader cmd4
    :initarg :cmd4
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorCommands (<MotorCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazypkg-msg:<MotorCommands> is deprecated: use crazypkg-msg:MotorCommands instead.")))

(cl:ensure-generic-function 'cmd1-val :lambda-list '(m))
(cl:defmethod cmd1-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:cmd1-val is deprecated.  Use crazypkg-msg:cmd1 instead.")
  (cmd1 m))

(cl:ensure-generic-function 'cmd2-val :lambda-list '(m))
(cl:defmethod cmd2-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:cmd2-val is deprecated.  Use crazypkg-msg:cmd2 instead.")
  (cmd2 m))

(cl:ensure-generic-function 'cmd3-val :lambda-list '(m))
(cl:defmethod cmd3-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:cmd3-val is deprecated.  Use crazypkg-msg:cmd3 instead.")
  (cmd3 m))

(cl:ensure-generic-function 'cmd4-val :lambda-list '(m))
(cl:defmethod cmd4-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:cmd4-val is deprecated.  Use crazypkg-msg:cmd4 instead.")
  (cmd4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCommands>) ostream)
  "Serializes a message object of type '<MotorCommands>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmd1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmd2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmd3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmd4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCommands>) istream)
  "Deserializes a message object of type '<MotorCommands>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmd1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmd2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmd3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmd4) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCommands>)))
  "Returns string type for a message object of type '<MotorCommands>"
  "crazypkg/MotorCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCommands)))
  "Returns string type for a message object of type 'MotorCommands"
  "crazypkg/MotorCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCommands>)))
  "Returns md5sum for a message object of type '<MotorCommands>"
  "1df7dce4d8fafaa23f1189b3eaa4180b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCommands)))
  "Returns md5sum for a message object of type 'MotorCommands"
  "1df7dce4d8fafaa23f1189b3eaa4180b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCommands>)))
  "Returns full string definition for message of type '<MotorCommands>"
  (cl:format cl:nil "float32 cmd1~%float32 cmd2~%float32 cmd3~%float32 cmd4~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCommands)))
  "Returns full string definition for message of type 'MotorCommands"
  (cl:format cl:nil "float32 cmd1~%float32 cmd2~%float32 cmd3~%float32 cmd4~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCommands>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCommands
    (cl:cons ':cmd1 (cmd1 msg))
    (cl:cons ':cmd2 (cmd2 msg))
    (cl:cons ':cmd3 (cmd3 msg))
    (cl:cons ':cmd4 (cmd4 msg))
))
