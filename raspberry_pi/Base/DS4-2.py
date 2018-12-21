import ctypes
test=ctypes.CDLL('./sendSPI.so')

import pygame
# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)
# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)
    def prin(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

pygame.init()

# Set the width and height of the screen [width,height]

#Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()
# -------- Main Program Loop -----------
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop

        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")


    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.

    textPrint.reset()
    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()

    textPrint.indent()

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()





# axis1:center axis2:左右(y軸回転) axis5:上下(x軸回転)
        axis1 = int(1000*joystick.get_axis( 1 ))
        axis2 = int(1000*joystick.get_axis( 2 ))
        axis5 = int(1000*joystick.get_axis( 5 ))



        buttonm = joystick.get_button( 2 )
        buttonb = joystick.get_button( 1 )
        button = buttonb + 2*buttonm




    test.trns(axis2,axis2,axis5,button)
    clock.tick(300)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()
