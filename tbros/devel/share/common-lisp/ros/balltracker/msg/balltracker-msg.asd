
(cl:in-package :asdf)

(defsystem "balltracker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ballcoords" :depends-on ("_package_ballcoords"))
    (:file "_package_ballcoords" :depends-on ("_package"))
    (:file "command" :depends-on ("_package_command"))
    (:file "_package_command" :depends-on ("_package"))
  ))