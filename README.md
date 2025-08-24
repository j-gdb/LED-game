# LED-game
This is a colour matching game made for the STM NucleoF446RE.

## Hardware
It uses 4 LED's, 4 buttons for each LED, and one RGB LED. They are hooked up with the corresponding to the labels in the "LED game.ioc" file. The RED, GREEN, and BLUE labels designate the RGB LED pins.

## How to play
The game starts when the user presses the user button. A light appears on the RGB LED, and you must press the button that corresponds to that LED colour. Then, the RGB LED will turn off and after a bit will turn on again.

## Future additions
- Might make it time reaction using Systick. I don't have a good way to display reaction times currently
