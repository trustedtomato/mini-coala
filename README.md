# mini COALA

## Getting ethernet to work

### Enable ethernet
`/boot/config.txt`\
dtoverlay=dwc2,dr_mode=host

`/boot/cmdline.txt`\
module-load=dwc2,g_ether

### Have static IP on the same subnet
On raspberry eth0: 192.168.1.10\
 255.255.255.0

On laptop ethernet: 192.168.1.50\
 255.255.255.0

## Developing website server & client
Go to the `website` directory.

### Setup
Run `npm install` to install all dependencies.

### Development
- Make sure `roscore` is running.
- Run `node ws-server.js` to start the WebSocket server.
- Run `npm run dev` to start the development server.

## Roadmap
- [ ] Measure thruster force
- [ ] Create node (publisher) for the barometer
- [ ] Create node (publisher) for the IMU
- [ ] Create node (subscriber) for the PCA
- [ ] Use the above nodes in the WebSocket node
- [ ] Figure out what we need on the webpage
  - [ ] Barometer data: display heave on the same graph as the target heave which is controlled on the webpage
  - [ ] IMU data: display roll, pitch, yaw, like here: https://codepen.io/fyodorio/pen/vWKxVW or STL!! https://threejs.org/examples/?q=stl#webgl_loader_stl
  - [ ] PCA data: display a bar for each thruster
  - [ ] Joystcik heave, and show the target heave on the same graph as the barometer data
  - [ ] Joystick yaw, and show the target pyramid, like the one in the IMU data
  - [ ] Joystick surge, and show a bar
  - [ ] Instead of the single line for the barometer, show a graph with the last 10 seconds of data
- [ ] Create the webpage design & communication