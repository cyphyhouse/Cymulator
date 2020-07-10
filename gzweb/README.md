# Gzweb - A web client for Gazebo

Gzweb is a WebGL client for [Gazebo](http://gazebosim.org).
Like gzclient, it's a front-end graphical interface to gzserver and provides visualization of the simulation.

Gzweb is same as a gzclient that it can be opened after gzserver is running on the machine.

This README is to help install Gzweb and visualize our experiment for CyPhyHouse project in Gzweb.

# Quick Start

## Step 1: Install Node.js v8.17.0

The newest version of Node.js is not supported by Gzweb.

1. Check the version of your own Node.js version
   ```bash
   node -v
   ```

2. List all the version of Node.js that can be installed
   ```bash
   nvm ls-remote
   ```

3. Choose a correct version Node.js and install it, v8.17.0 or other older version is recommended
   ```bash
   nvm install 8.17.0
   ```

4. After installation, remember to switch the version of Node.js because the system default is to use the newest version
   ```bash
   nvm use 8.17.0
   ```


## Step 2: Install Gzweb

Gzweb is only a gui tool to show the result of simulation,
so the installation process for Gzweb is the same whether it is used with Gazebo 7 or above.
Here we install Gzweb with Gazebo 7.

1. Before install Gzweb, run the following to install dependencies including Node.js and Gazebo.
Because the older version of node is recommended, do check the version of Node.js after this step.
You can skip this step if you already install Gazebo.
   ```bash
   sudo apt install gazebo7 libgazebo7-dev

   sudo apt install libjansson-dev nodejs npm nodejs-legacy \
       libboost-dev imagemagick libtinyxml-dev mercurial cmake \
       build-essential
   ```

2. Clone the repository for Gzweb 1.4.0.
    ```bash
    git clone https://github.com/osrf/gzweb.git --branch gzweb_1.4.0
    ```

3. Before running the deploy script, it's important to source the Gazebo `setup.sh` file
   ```bash
   # If you installed Gazebo via apt
   source /usr/share/gazebo/setup.sh

   # If you did a source install then
   source <YOUR_GAZEBO_PATH>/share/gazebo/setup.sh
   ```

4. Download models to Gzweb, you'll need to gather all the Gazebo models which you want to simulate in the right directory (`http/client/assets`) and prepare them for Gzweb.
   ```bash
   # To skip downloading models from the model database and grab only local models in your Gazebo model path, do
   npm run deploy --- -m local

   # To generate thumbnails for all models, run the script with the -t flag, i.e.,
   npm run deploy --- -t
   ```

## Step 3: Running Gzweb

Again, you need to use the older version of Node.js to run Gzweb.
For example, if you install the Node.js v8.17.0 in Step 1, and  build the Gzweb with `npm` in Step 2,
you need to use the same version in this step as well,
or there can be unexpected errors.

1. On the server machine, start gazebo or gzserver first
   ```bash
   # Example command to spawn a Hector Quadrotor in an empty world
   roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch gui:=false
   ```

2. Open another terminal, from the Gzweb directory run the command to start the gzclient.
   ```bash
   npm start
   ```

3. Open a internet browser with WebGL and websocket support.
   Type in the IP address and port.
   The default HTTP address is at `http://localhost:8080`

4. To stop `gzserver`, press `Ctrl+C` to terminate the command.



# Trouble shooting

1. Compilation error when running `npm run deploy --- -m`

   Maybe using the wrong version of the Node.js.

2. Error when running `npm start`

   You need to make sure the version of Node.js you are using are the same version when you install Gzweb

3. The web browser stopped working.

   Try different web browsers.
   Firefox may not work well, and Chrome is recommended.

4. The model is missing when opening Gzweb.

   You need to manually copy the model folder into `gzweb/http/client/assets`.


## How to find the correct folder that stores the assets

The asset usually has a meshes folder in it.
Here's an example for the asset folder of Hector Quadrotor used in CyPhyHouse which is under `hector_quadrotor_description`.

```bash
> ls

CHANGELOG.rst  CMakeLists.txt  launch/  meshes/ package.xml  urdf/
```
