#include <SFML/Graphics.h>

int main()
{
	sfVideoMode mode = { 800, 600, 32 };  // width, height, bits per pixel
	/* Create the main window */
	sfRenderWindow* main_window = sfRenderWindow_create(mode, "SFML window", sfResize | sfClose, NULL);
	if (!main_window)
		return 1;

	sfCircleShape* circle = sfCircleShape_create();
	sfCircleShape_setFillColor(circle, sfGreen);
	sfCircleShape_setRadius(circle, 100);

	sfVector2f v1 = { 200, 200 };
	sfVector2f v2 = { 300, 300 };
	sfVertex p1 = { v1, sfRed, v1 };
	sfVertex p2 = { v2, sfRed, v2 };
	sfVertexArray* line = sfVertexArray_create();
	sfVertexArray_setPrimitiveType(line, sfLines);
	sfVertexArray_append(line, p1);
	sfVertexArray_append(line, p2);

	
	sfEvent event;
	sfView* new_vew = sfView_create();
	while (sfRenderWindow_isOpen(main_window)) {
		while (sfRenderWindow_pollEvent(main_window, &event)) {
			if (event.type == sfEvtClosed) {
				sfRenderWindow_close(main_window);
			}
		}
		sfRenderWindow_clear(main_window, sfTransparent);
		sfRenderWindow_drawCircleShape(main_window, circle, NULL);
		sfRenderWindow_drawVertexArray(main_window, line, NULL);
		sfRenderWindow_display(main_window);
	}

	sfCircleShape_destroy(circle);

	return 0;
}