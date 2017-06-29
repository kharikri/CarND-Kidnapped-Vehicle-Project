# Websocket installation instructions for Windows 10

Installing Websockets on Windows 10 is not straight forward. After going through several posts on the Udacity SDC forums and on the Internet, the following steps worked for me.

This assumes [Linux Bash Shell](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for Windows 10 is installed. Run the following commands inside the Linux Bash Shell:
1. sudo apt-get update
2. sudo apt-get install git
3. sudo apt-get install cmake
4. sudo apt-get install openssl
5. sudo apt-get install libssl-dev
6. git clone <<https://github.com/udacity/CarND-Kidnapped-Vehicle-Project>> or whatever CarND project
7. sudo rm /usr/lib/libuWS.so 
8. cd to wherever your CarND-Kidnapped-Vehicle-Project/ is
9. ./install-ubuntu.sh

Step 9 may fail for number of reasons as listed below:

A. install-ubuntu.sh has only rw but no x permission. Run "chmod +x install-ubuntu.sh" to give execution permission  
B. Cannot find the package "libuv1-dev"

	* To install the package run "sudo apt-get install libuv1.dev"
	* If you still cannot install the package run the following to get the package and install it:  
		sudo add-apt-repository ppa:acooks/libwebsockets6  
		sudo apt-get update  
		sudo apt-get install libuv1.dev	  	
C. May complain about the version of cmake you have. You need a version greater than 3.0. [Here](https://askubuntu.com/questions/355565/how-to-install-latest-cmake-version-in-linux-ubuntu-from-command-line) is a link which describes how to get version 3.8. Look at Teocci's response in this link  
D. Installing cmake requires g++ compiler. Install a g++ version 4.9 or greater. Here are the steps:
	sudo add-apt-repository ppa:ubuntu-toolchain-r/test  
	sudo apt-get update  
	sudo apt-get install g++-4.9  
