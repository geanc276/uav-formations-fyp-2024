# Installing tmux on Ubuntu

`tmux` is a useful terminal splitting program which helps run simulation and field tests for the drone swarm. 

### 1. Install Tmux

Open a terminal and run the following commands to update the package list and install Tmux:

```bash
sudo apt update
sudo apt install tmux
```

### 2. Install Tmux Plugin Manager (TPM)

TPM makes it easy to manage tmux plugins. Run these commands to install TPM:

```bash
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
```

### 3. Create or Edit the .tmux.conf File:

If you don't already have a .tmux.conf file, create one, and open it:

```bash
touch ~/.tmux.conf
nano ~/.tmux.conf
```

### 4 Add Configuration for Mouse support and key Bindings

Add the following lines to your `~/.tmux.conf` file: 

```bash
# Initialize Tmux Plugin Manager
set -g @plugin 'tmux-plugins/tpm'

# Enable mouse support
set -g mouse on

# Split horizontally with Ctrl + h
bind-key -n C-h split-window -h

# Split vertically with Ctrl + v
bind-key -n C-v split-window -v
```

### 5. Start Tmux:

Simply start a new tmux session by running:

```bash
tmux
```

You should now have tmux running with mouse support and the ability to split horizontally with Ctrl + h and vertically with Ctrl + v. You can also click on panes to switch between them.