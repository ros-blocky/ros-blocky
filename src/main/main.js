// Modules to control application life and create native browser window
const { app, BrowserWindow } = require('electron');
const { initialize } = require('./windows/setupWindow');
const { registerProjectIPC } = require('./ipc/mainIPC');

// Register IPC handlers when app is ready
app.whenReady().then(() => {
  console.log('App is ready');

  // Register project management IPC handlers
  registerProjectIPC();

  initialize();

  app.on('activate', function () {
    // On macOS it's common to re-create a window in the app when the
    // dock icon is clicked and there are no other windows open.
    if (BrowserWindow.getAllWindows().length === 0) {
      initialize();
    }
  });
});

// Quit when all windows are closed, except on macOS. There, it's common
// for applications and their menu bar to stay active until the user quits
// explicitly with Cmd + Q.
app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') app.quit();
});
