# Introduction to Open-Source Robotics

These instructions are intended for contributors. If you want to read the book,
it *lives* here: https://crigroup.gitbooks.io/osrobotics/content/

## Requirements
Installing GitBook is easy and straightforward. You just need:
* NodeJS (v4.0.0 and above is recommended)

### Install and configure `nodejs`
You can install `nodejs` from the Ubuntu package repositories:
```bash
sudo apt-get install nodejs
```

Now, let's create a symbolic link so that we can use the `npm` command:
```bash
sudo ln -s /usr/bin/nodejs /usr/bin/node
```

Create a directory for the packages:
```bash
mkdir -p ~/.npm_packages
```

Indicate to `npm` where to store the packages. In your `~/.npmrc` file add:
```
prefix=~/.npm_packages
```

Ensure `npm` will find installed binaries and man pages. Add the following to
your `~/.bashrc` file:
```bash
NPM_PACKAGES="~/.npm_packages"
PATH="$NPM_PACKAGES/bin:$PATH"
# Unset manpath so we can inherit from /etc/manpath via the `manpath` command
unset MANPATH # delete if you already modified MANPATH elsewhere in your config
export MANPATH="$NPM_PACKAGES/share/man:$(manpath)"
```

Finally, source your `~/.bashrc` file manually:
```bash
source ~/.bashrc
```

### Install and configure `gitbook`
 Simply run the following command to install GitBook
```bash
sudo apt-get install npm
sudo npm install gitbook-cli -g
```

### Clone this repository
```bash
mkdir -p ~/git; cd ~/git
git clone https://github.com/crigroup/osrobotics.git
```

## Build and serve locally
```
cd ~/git/osrobotics
gitbook install
gitbook serve
```

If everything is right, the website will be *served* here: http://localhost:4000

## Upload to osrobotics.org
```
rsync -r -a -v -e  "ssh -p2222" --delete _book/ user@server:osrobotics/osr/
```

