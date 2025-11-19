import pygame

pygame.init()
# set the screen dimensions
screen_width = 600
screen_height = 800
screen = pygame.display.set_mode((screen_width, screen_height))
# set caption fot the screen title if required
pygame.display.set_caption("The main screen")

# define colors
black = (0, 0, 0)
white = (255, 255, 255)
# define your font size and type
my_font = pygame.font.SysFont('Times new roman', 20)
# specify the text to be displayed
my_text = "Robotics is the future "
# render the  text surface
text_surface = my_font.render(my_text, True, white)
# get the triangle that encloses the text and position it
text_rect = text_surface.get_rect()
text_rect = (4, 4)

# main game loop(drawing on the screen)
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # drawing steps
    screen.fill(black)
    # blit/draw the text surface on the main screen surface
    screen.blit(text_surface, text_rect)
    # update the display to show everything we just drew
    pygame.display.flip()  # or pygame.display.update

# quit pygame
pygame.quit()
