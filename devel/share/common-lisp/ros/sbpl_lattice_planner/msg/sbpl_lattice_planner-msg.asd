
(cl:in-package :asdf)

(defsystem "sbpl_lattice_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SBPLLatticePlannerStats" :depends-on ("_package_SBPLLatticePlannerStats"))
    (:file "_package_SBPLLatticePlannerStats" :depends-on ("_package"))
  ))