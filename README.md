# Quick Start on Ubuntu 16.04
Install git:

    $ sudo apt-get install git

Add your ssh key to clone the repository.

    User->Settings->SSH Keys

Clone the repo:
    
    mkdir ~/git/; cd ~/git/
    git git@animal.informatik.uni-stuttgart.de:practical_course_ss17/mlr.git
    
Once the repo has been cloned:

    cd mlr
    git checkout roopi
	./INSTALL_ALL_UBUNTU_PACKAGES.sh
    ./INSTALL_ROS_PACKAGES_KINETIC
    ./bin/createMakefileLinks.sh
    cd examples/Roopi/basic/
    make
