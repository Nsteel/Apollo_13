Wie man dieses Paket verwenden kann:

Grundsätzlich bieten wir folgende fertig implementierte Funktionen:

    1.  Fernsteuerung des Roboters über die Smartphone App "TouchOSC",
        das beinhaltet:
            - RC-Mode (klassische ferngesteuerte Lenkung)
            - einschalten der autonomen Zielführung mit vorgegebenen Zielen
              aus Rviz
            - einschalten der autonomen Durchfahrt durch mit zwei Aruco-Markern
              markierten Tore.

    2.  Autonome Zielführung mit vorgegebenen Zielenaus Rviz.
        ->  Man gibt in Rviz mit "set 2D-Goal" ein Ziel vor, mit sbpl wird ein
            kollisionsfreier Weg zum Ziel geplant. Hat man dann mit der App
            die autonome steuerung freigegeben, fängt das Auto an sich dorthin
            zu bewegen.

    3.  Autonome Durchfahrt durch mit zwei Aruco-Markern markierte Tore.
        ->  Hat das Auto mindestens zwei Marker entdeckt, kann man mit der
            Freigabe durch die App die automatische Durchfahrt aktivieren.
            Das tut der Roboter solange automatische bis keine Tore mehr zum
            durchfahren gefunden werden können.

    4.  Aufzeichnung der Umgebung mit der Kinect Kamera und Erstellung einer Map.

Aufrufen der Funktionen:

    Zu 1.)
        - Folgende Befehle in die Konsole eingeben:
        - sudo su
        - roslaunch apollo_13 apollo_13.launch <Parameter>

          Parameter:  kinect=true/false(default), gui=true(default)/false
          Bsp:  roslaunch apollo_13 apollo_13.launch kinect:=true

        - Das Smartphone muss sich im selben Netzwerk befinden wie der Roboter
          und TouchOSC muss installiert und gestartet sein.
        - jetzt kann man das auto fernsteuern

    Zu 2.)
        - Folgende Befehle in die Konsole eingeben:
        - sudo su
        - roslaunch apollo_13 apollo_13.launch kinect:=true gui:=false
        - wenn der Startvorgang abgeschlossen ist, neues Konsolenfenster:
        - roslaunch apollo_13 car_display.launch

        - Wie in 2.) oben beschrieben kann man durch setzen eines Ziels auf der
          Karte die Wegplanung in Gang setzen und durch die App die Zielführung
          mit einem Druck auf "autonomous mode" aktivieren.

    Zu 3.)
        - Folgende Befehle in die Konsole eingeben:
        - sudo su
        - roslaunch apollo_13 apollo_13.launch kinect:=true gui:=false
        - wenn der Startvorgang abgeschlossen ist, neues Konsolenfenster:
        - roslaunch apollo_13 aruco_boards.launch
        - wenn der Startvorgang abgeschlossen ist, neues Konsolenfenster:
        - roslaunch apollo_13 car_display.launch

        - Wie in 3.) oben beschrieben kann man sobald das Fahrzeug zwei Aruco-
          Marker entdeckt hat (grüne Zylinder mit Pfeil auf der Karte) mit
          einem Druck auf "Follow Aruco markers" die Zielführung beginnen.
          Es kann etwas dauern bis nach Einschalten der Funktion ein Ziel
          gefunden wurde. Ist eines gefunden worden erscheint ein roter 2D goal
          Marker.

    Zu 4.)
        - Folgende Befehle in die Konsole eingeben:
        - roslaunch apollo_13 mapping.launch
	- nach dem mappen folgendes in die Konsole eingeben:
	- rosrun map_server map_saver

        - Zu Beachten:
            - Solange der Drehgeber noch defekt ist, nicht schneller als Stufe 2
              vorwärts fahren oder -3 rückwärts.
            - Winkelgeschwindigkeiten minimieren, d.h. kurven langsam fahren oder
              große Lenkeinschläge vermeiden.
            - Nicht länger als unbedingt notwendig auf der Stelle stehen bleiben,
              da der IMU trotz Filterung irgendwann das "klettern" anfängt und sich
              der Raum aus Sicht des Roboters zu drehen beginnt.
            - gerade Gänge auch gerade durchfahren, wegen der bregrenzten Sicht der
              Kinect werden sonst Wandstücke aneinander gesetzt, das ist nicht immer
              perfekt und kann zu leichten Krümmungen in langen Gängen führen.
            - Tips zum Mappen unter diesem Link:
              http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

	-Bemerkungen:
	   - Sollten beim Start eines launchfiles fehler auftreten, hilft oft der Reset des
	     Arduinos.
	   - Ebenso treten Fehlermeldungen bei einem kritischen Batteriestand der 
	     Kinect auf.
