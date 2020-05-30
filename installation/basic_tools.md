# Basic tools: Ubuntu, Python, Git

## Ubuntu

1. Download [Ubuntu 16.04 Desktop](http://releases.ubuntu.com/16.04.6/).
2. Create a Bootable Thumbdrive. You can use this
[Universal USB installer](https://www.pendrivelinux.com/universal-usb-installer-easy-as-1-2-3/)
3. Shutdown your computer and put the thumbdrive in a USB slot.
4. Change the Boot Sequence so that your thumbdrive is booted first.
5. The Ubuntu installer should start up after the computer restarts. Follow the
[installation instructions](https://www.ubuntu.com/download/desktop/install-ubuntu-desktop)
to your specific needs.

## Python

You can install python from the Ubuntu package repositories. Run the following
commands in a terminal:
{% label %}command-line{% endlabel %}
```bash
sudo apt-get update
sudo apt-get install ipython python-dev python-numpy python-pip python-scipy
```

To test your installation, run the following commands:
{% label %}command-line{% endlabel %}
```bash
python -c "import IPython; print('IPython v{}'.format(IPython.__version__))"
# IPython v2.4.1
python -c "import numpy; print('numpy v{}'.format(numpy.__version__))"
# numpy v1.11.0
python -c "import scipy; print('scipy v{}'.format(scipy.__version__))"
# scipy v0.17.0
```

## Git

Git is sophisticated version control software. First, you should create an
account in [GitHub](https://github.com/).

Next, install Git from the Ubuntu package repositories
{% label %}command-line{% endlabel %}
```bash
sudo apt-get update
sudo apt-get install git
```

Set-up your Github details
{% label %}command-line{% endlabel %}
```bash
git config --global user.name "your-github-username"
git config --global user.email "your-email@address.com"
```

You can now clone the course repository
{% label %}command-line{% endlabel %}
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/crigroup/osr_course_pkgs.git
```
