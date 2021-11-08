Clone this directory into your catkin_ws/src folder (this file should be at ~/catkin_ws/src/WPIBotOnAWire/README.md)

Commands:

    bow-term: starts a new terminal into a docker running ros. Automagically starts/downloads/runs dependencies, run this usually.
    bow-make: runs catkin_make.
    bow-node: runs a ros node in docker. eg. to start a python node at mynode/app.py, run 'bow-node mynode app.py'
    bow-echo: echos a ros topic from the docker. eg. to echo topic /mytopic, run bow-echo /mytopic
    bow-start: starts a docker container running docker without opening a terminal into it.
    bow-stop: stops the docker container

To use commands, source your shell's specific file before use:

For bash, (Ubuntu's default shell) do: 
    
    source ~/catkin_ws/src/WPIBotOnAWire/source.sh

You will have to do this command every time you start a new terminal. Alternatively, add the script to your
shell's startup file.

For bash, run:

    echo "source ~/catkin_ws/src/WPIBotOnAWire/source.sh" > ~/.bashrc