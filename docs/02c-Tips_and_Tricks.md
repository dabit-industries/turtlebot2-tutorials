## Tips and Tricks
The following is a collection of useful tips and tricks that would help speed up aspects of connecting to and setting up the Turtlebot.

## Connecting to the Turtlebot remotely from the Master Computer

1. Open a new terminal on the Master Computer
    1. `ssh turtlebot@IP_OF_TURTLEBOT`


## Using a shell window manager
Byobu, Tmux, Screen
 

## Using Bash Aliases
[~/.bash_aliases](/Setup/.bash_aliases)  
[~/.rosrc](/Setup/.rosrc)  

at bottom of .bashrc:
```bash
if [ -f ~/.rosrc ]; then
  . ~/.rosrc
fi
```


[Return to the main README page](/README.md)
