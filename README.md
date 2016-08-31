# Wie man dieses Paket verwenden kann:

### Grundsätzlich bieten wir folgende fertig implementierte Funktionen:
1. **Fernsteuerung des Roboters über die Smartphone App "TouchOSC", das beinhaltet:**
    - RC-Mode (klassische ferngesteuerte Lenkung)
    - einschalten der autonomen Zielführung mit vorgegebenen Zielen aus Rviz
    - einschalten der autonomen Durchfahrt durch mit zwei Aruco-Markern markierten Tore.

    - **_Zu Beachten_**:
        - die Smartphone App ist kostenpflichtig und wurde von der Firma Hexler entwickelt.
        - Die Apollo_13 Vorlage für TouchOSC ist in `apollo_13/data/TouchOSC_Template` gespeichert.
        - Unsere Vorlage kann mit dem kostenlosen TouchOSC Editor geändert werden. 

        - Mehr Information zu TouchOSC und dem Editor unter:
                [http://hexler.net/software/touchosc](http://hexler.net/software/touchosc) 

2.  **Autonome Zielführung mit vorgegebenen Zielenaus Rviz.**
    -  Man gibt in Rviz mit "set 2D-Goal" ein Ziel vor, mit sbpl wird ein
            kollisionsfreier Weg zum Ziel geplant. Hat man dann mit der App
            die autonome steuerung freigegeben, fängt das Auto an sich dorthin
            zu bewegen.

3.  **Autonome Durchfahrt durch mit zwei Aruco-Markern markierte Tore.**
    -   Hat das Auto mindestens zwei Marker entdeckt, kann man mit der
            Freigabe durch die App die automatische Durchfahrt aktivieren.
            Das tut der Roboter solange automatische bis keine Tore mehr zum
            durchfahren gefunden werden können.

4.  **Aufzeichnung der Umgebung mit der Kinect Kamera und Erstellung einer Map.**

## Aufrufen der Funktionen:

- **Zu 1.)**
    1. Folgende Befehle in die Konsole eingeben:
      ```bash
      sudo su
      roslaunch apollo_13 apollo_13.launch <Parameter>
      ```
      - Parameter:  `kinect:=true/false(default), gui:=true(default)/false`
        - Beispiel: `roslaunch apollo_13 apollo_13.launch kinect:=false`
        
    2. Jetzt kann man das auto fernsteuern
    
    - **_Bemerkung:_** Das Smartphone muss sich im selben Netzwerk befinden wie der Roboter
      und TouchOSC muss installiert und gestartet sein.

- **Zu 2.)**
    1. Folgende Befehle in die Konsole eingeben:
    ```bash
      sudo su
      roslaunch apollo_13 apollo_13.launch kinect:=true gui:=false
      ```
    2. Wenn der Startvorgang abgeschlossen ist, neues Konsolenfenster:
    3. `roslaunch apollo_13 car_display.launch`
    
    - Wie in _2.)_ oben beschrieben kann man durch setzen eines Ziels auf der
      Karte die Wegplanung in Gang setzen und durch die App die Zielführung
      mit einem Druck auf "autonomous mode" aktivieren.

- **Zu 3.)**
    1. Folgende Befehle in die Konsole eingeben:
    ```bash
      sudo su
      roslaunch apollo_13 apollo_13.launch kinect:=true gui:=false
      ```
    2. Wenn der Startvorgang abgeschlossen ist, neues Konsolenfenster:
    3. `roslaunch apollo_13 aruco_boards.launch`
    4. wenn der Startvorgang abgeschlossen ist, neues Konsolenfenster:
    5. `roslaunch apollo_13 car_display.launch`

    - Wie in _3.)_ oben beschrieben kann man sobald das Fahrzeug zwei Aruco-
      Marker entdeckt hat (grüne Zylinder mit Pfeil auf der Karte) mit
      einem Druck auf "Follow Aruco markers" die Zielführung beginnen.
      Es kann etwas dauern bis nach Einschalten der Funktion ein Ziel
      gefunden wurde. Ist eines gefunden worden erscheint ein roter 2D goal
      Marker.

    - **_Zu Beachten_**:
        - Jeder Aruco-Marker hat eine eindeutige ID und wird nur vom Node car_aruco 
            als gültig erkannt, wenn seine ID in einer yml Datei gespeichert wird 
            und ihr Pfad in `apollo_13/config/aruco_boards/boards.yml` hinterlegt ist.
            Schauen sie sich am besten die vorhandenen Marker-Beschreibungen 
            in `apollo_13/config/aruco_boards/` an. Hierbei ist nur die ID zu ändern. 

- **Zu 4.)**
  1. Folgende Befehle in die Konsole eingeben:
    - `roslaunch apollo_13 mapping.launch`
  2. Nach dem Mappen folgendes in die Konsole eingeben:
    - `rosrun map_server map_saver`

  - **_Zu Beachten_**:
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
      [http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData)

#### Bemerkungen:
   - Sollten beim Start eines launchfiles fehler auftreten, hilft oft der Reset des
       Arduinos.
   - Ebenso treten Fehlermeldungen bei einem kritischen Batteriestand der 
       Kinect auf.
