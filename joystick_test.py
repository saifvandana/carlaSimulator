import pygame

pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick: {joystick.get_name()}")
print(f"Number of buttons: {joystick.get_numbuttons()}")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed")
        elif event.type == pygame.JOYBUTTONUP:
            print(f"Button {event.button} released")

pygame.quit()