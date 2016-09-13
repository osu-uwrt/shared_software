### Underwater ROS

---

__The Underwater Robotics Team__  
_Vehicle Agnostic Packages & Utilities_

---

`osu-uwrt` -- A Python script for installing and configuring our development environment.

    osu-uwrt install ceres
    osu-uwrt install ros
    source ~/.bashrc
    osu-uwrt install riptide
    source ~/.bashrc
    osu-uwrt uninstall ros

---

`/scripts`  -- The original scripts upon which `osu-uwrt` is based.

---

### To-do:
* Logging
  * Status persistence
  * Subprocess output
  * Error descriptions
* Sphinx
  * OSX/Windows compatibility
* Uninstall
  * Incorporate logged status
* `configure-udev` --> `osu-uwrt`
  * Add sensors & microcontrollers
* `install-avr` --> `osu-uwrt`
  * Testing & competition updates
