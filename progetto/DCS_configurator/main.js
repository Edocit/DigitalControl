
const electron     = require('electron');
const fs           = require('fs'); 

const { app, BrowserWindow, Menu, MenuItem, ipcMain } = require('electron');

let win;
let buff;



//creo la barra del menu 
const template = [
    {
        label: 'Contact developer',
            submenu: [
                { label: 'Ask a question', },
                { label: 'Report problem', }
            ]
    },
    { label: 'Info', click(){ display_info_window(); } },
    {
        label: 'help',
            submenu: [
                { label: 'Documentation', },
                { label: 'FAQ', }
            ]
    }
]


//reading target file for the application 
function read_file(){
    buff = fs.readFileSync('common.h','utf-8');
}





const menu = Menu.buildFromTemplate(template);


function createWindow(pyshell){
    // Crea la finestra del browser
    win = new BrowserWindow({
      width: 1000,
      height: 820,
      resizable: false,
      center: true,

      webPreferences: {
        nodeIntegration: true
      }
    })

    Menu.setApplicationMenu(menu);
    win.webContents.openDevTools();
    win.loadFile('apparence.html');
}
  


function display_info_window(){
    let info_window = new BrowserWindow({ 
        parent: win, 
        modal: true, 
        show: true,
        width: 500,
        height: 300,
        autoHideMenuBar: true,
        resizable: false,
        center: true
    });

    info_window.loadFile('info.html');
}






app.whenReady().then(createWindow);