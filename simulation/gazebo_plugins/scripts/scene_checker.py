#!/usr/bin/python3

'''
    Copyright (C) 2018 João Borrego

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    
        http://www.apache.org/licenses/LICENSE-2.0
        
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
'''

# GUI
import tkinter as tk
from PIL import Image, ImageTk
# Command line args
import sys, getopt
# XML parsing
import xml.etree.ElementTree as ET

# Title
APP_TITLE = "Scene dataset verification tool"
# Usage
USAGE = 'options: -i <image output directory>\n' +                         \
        '         -d <dataset output directory>\n' +                       \
        '         -s <number of scenes> [optional, default=1]\n' +         \
        '         -n <index of the first scene> [optional, default=0]\n' + \
        '         -e <image file extension> [optional, default=.png]'

# Dataset file extension
EXT_DATA = '.xml'
# Export directory
EXPORT_DIR = 'export/'
# Show help command
SHOW_HELP = True

# Sphere color
SPHERE_COLOR = 'red'
# Box color
BOX_COLOR = 'blue'
# Cylinder color
CYLINDER_COLOR = '#36d129'
# Text color
TEXT_COLOR = 'white'

def parseArgs(argv):
    '''
    Parses command-line options.
    '''

    # Parameters
    data_dir = ''
    img_dir = ''
    first = 0
    scenes = 1
    img_ext = '.png'

    usage = 'usage:   ' + argv[0] + ' [options]\n' + USAGE

    try:
        opts, args = getopt.getopt(argv[1:],
            "hi:d:s:n:e:",["img_dir=","data_dir=","scenes=","first=","img_ext"])
    except getopt.GetoptError:
        print (usage)
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print (usage)
            sys.exit()
        elif opt in ("-i", "--img_dir"):
            img_dir = arg
        elif opt in ("-d", "--data_dir"):
            data_dir = arg
        elif opt in ("-s", "--scenes"):
            scenes = int(arg)
        elif opt in ("-n", "--first"):
            first = int(arg)
        elif opt in ("-e", "--img_ext"):
            img_ext = arg
    
    if not data_dir or not img_dir:
        print (usage)
        sys.exit(2)

    print ('Image directory      ', img_dir)
    print ('Dataset directory    ', data_dir)
    print ('Number of scenes     ', scenes)
    print ('Index of first scene ', first)
    print ('Image file extension ', img_ext)

    return [data_dir, img_dir, scenes, first, img_ext]

class ImageViewer(tk.Frame):
    '''
    Displays scene image with bounding box and additional info overlays.
    '''

    def __init__(self, parent, data_dir, img_dir, scenes, first, img_ext):
        '''
        Initializes attributes
        '''
        tk.Frame.__init__(self, parent)
        self.parent = parent
        self.data_dir = data_dir
        self.img_dir = img_dir
        self.scenes = scenes
        self.first = first
        self.cur = first
        self.img_ext = img_ext

        self.setupWidgets()

    def setupWidgets(self):
        '''
        Sets up GUI elements and event callback funtions
        '''

        # Configure window
        self.parent.maxsize(
            width = self.parent.winfo_screenwidth(),
            height= self.parent.winfo_screenheight())
        self.parent.title(APP_TITLE)

        # Create canvas in main window
        self.canvas = tk.Canvas(self.parent,
            width =self.parent.winfo_screenwidth(),
            height=self.parent.winfo_screenheight())
        self.canvas.pack()

        # Bind events
        self.parent.bind("<Left>", self.onLeft)
        self.parent.bind("<Right>", self.onRight)
        self.parent.bind("<Return>", self.onQuit)
        self.parent.bind("<space>", self.onSave)

        # Update image
        self.onUpdate()

    def onUpdate(self):
        '''
        Shows image and object bounding box overlays.
        '''

        # Filenames
        img_file = self.img_dir + '/' + str(int(self.cur / 100)) + \
            '00/' + str(self.cur) + self.img_ext
        data_file = self.data_dir + '/' + str(self.cur) + EXT_DATA
        # print(img_file, data_file)

        # Clean canvas
        self.canvas.delete("all")

        # Update scene image
        try:
            image = Image.open(img_file) 
            photo = ImageTk.PhotoImage(image)
        except:
            print('Could not open ' + img_file + '.Exiting...')
            self.parent.destroy()
            sys.exit(2)

        self.canvas.image = photo
        self.canvas.create_image(0, 0, image=photo, anchor=tk.NW)

        # Open XML dataset file
        try:
            tree = ET.parse(data_file)
        except:
            print('Could not open ' + data_file + '. Exiting...')
            self.parent.destroy()
            sys.exit(2)

        annotation = tree.getroot()

        if SHOW_HELP:
            # Commands
            self.canvas.create_rectangle(40, 40, 300, 230,
                fill="gray", stipple="gray50", width=0)
            # Draw label with cur / total scene indicator
            label = 'Scene ' + str(self.cur) + '/' + str(self.first + self.scenes - 1)
            self.canvas.create_text(45, 45, text=label, fill="white",
                font=('arial', '18'), anchor=tk.NW)
            # Draw help label
            label = 'Commands\n'            + \
                    'Left\tPrevious\n'      + \
                    'Right\tNext\n'         + \
                    'Spacebar\tExport\n'    + \
                    'Return\tExit'
            self.canvas.create_text(45, 80, text=label, fill="white",
                font=('arial', '18'), anchor=tk.NW)

        # Draw bounding boxes
        for obj in annotation.findall('object'):
            name = obj.find('name').text
            color = BOX_COLOR
            if   (name == "sphere"):   color = SPHERE_COLOR
            elif (name == "cylinder"): color = CYLINDER_COLOR
            
            for bnd_box in obj.findall('bndbox'):
                x_min = int(bnd_box.find('xmin').text)
                y_min = int(bnd_box.find('ymin').text)
                x_max = int(bnd_box.find('xmax').text)
                y_max = int(bnd_box.find('ymax').text)
                rect = self.canvas.create_rectangle(x_min, y_min, x_max, y_max,
                    outline=color, width=2)
                text = self.canvas.create_text(x_min, y_min-25, text=name,
                    fill=TEXT_COLOR, font=('arial', '16'), anchor=tk.NW)
                background = self.canvas.create_rectangle(self.canvas.bbox(text),
                    outline=color, fill=color)
                self.canvas.tag_lower(background, text)

    def onSave(self, event):
        
        export_name = "export/" + str(self.cur) + ".ps"
        self.canvas.postscript(file=export_name)
        print('Exported ' + export_name)

    def onLeft(self, event):
        '''
        Updates counter and calls update function.
        '''
        if (self.cur == 0): return

        self.cur = self.cur - 1
        if (self.cur >= self.first + self.scenes): sys.exit(0) 
        self.onUpdate()

    def onRight(self, event):
        '''
        Updates counter and calls update function.
        '''
        if (self.cur == self.first + self.scenes - 1): return

        self.cur = self.cur + 1
        if (self.cur >= self.first + self.scenes): sys.exit(0) 
        self.onUpdate()

    def onQuit(self, event):
        '''
        Quits program.
        '''
        self.parent.destroy()
        sys.exit(0)

def main(argv):
    '''
    Simple tool to open image and overlay bounding box data to check
    whether or not a scene dataset is correct.
    '''
    
    # Obtain command-line arguments
    [data_dir, img_dir, scenes, first, img_ext] = parseArgs(argv)

    # Open root window
    root = tk.Tk()
    # Create app object
    app = ImageViewer(root, data_dir, img_dir, scenes, first, img_ext)
    # Main loop
    root.mainloop()

if __name__ == "__main__":
   main(sys.argv)
