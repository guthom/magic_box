import os, sys

#find better way to do this.
projectDir = os.path.join(os.path.dirname(__file__), '../')
test = os.path.dirname(__file__)
displaySources = os.path.join(projectDir, "../thirdparty/e-Paper/RaspberryPi&JetsonNano/python/lib/waveshare_epd")
sys.path.append(displaySources)

import epd4in2 as display
import PIL.Image as Image
import numpy as np

class EInkDisplay:

    width = 0
    heigth = 0
    edp = None

    def __init__(self, width=400, heigth=300):
        self.width = width
        self.heigth = heigth
        self.InitDisplay()

    def ResizeTag(self, image):
        factor = float(self.heigth) / float(self.width)

        image = image.resize((self.width, self.heigth), Image.NEAREST)

        #produce aspect ratio we want
        padNumber = int(self.width * factor)
        pix = np.array(image)
        pix = np.pad(pix, ((0, 0), (padNumber / 2, 0), (0, 0)), 'constant')

        pix = np.roll(pix, -padNumber/4, axis=1)
        image = Image.fromarray(pix)

        image = image.resize((self.width, self.heigth), Image.NEAREST)
        return image

    def InitDisplay(self):
        self.epd = display.EPD()
        self.epd.init()

    def ShowImage(self, image):

        image = self.ResizeTag(image)
        self.epd.display_frame(self.epd.get_frame_buffer(image))
