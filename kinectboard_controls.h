// vim:ts=4:sw=4:expandtab
/*
 * Kinect Board Controls 
 */

#ifndef __KINECTBOARD_CONTROLS
#define __KINECTBOARD_CONTROLS
 
 #include <sys/types.h>
 #include <SDL/SDL.h>
 #include <stdbool.h>
 
/* ******************************************************************* */
/* Utility function */
SDL_Surface* kb_surface_fill_color(SDL_Rect* box, SDL_Color* color);

/* ******************************************************************* */
/* Control list structure */

// Types
typedef enum
{
	KB_BUTTON = 0,
	KB_SLIDER
} kb_type;

#pragma pack(push)
typedef struct
{
	void* parent;
	void* data;
	void* next;
	kb_type type;
} kb_controls_node;
#pragma pack(pop)

#pragma pack(push)
typedef struct 
{
	kb_controls_node* root;
	uint32_t size;
} kb_controls;
#pragma pack(pop)

// Create list of controls
kb_controls* kb_controls_create();
// Destroy list of controls
void kb_controls_destroy(kb_controls* ptr);

// Render all controls in the list
void kb_controls_render(kb_controls* list, SDL_Surface* screen);

// Add a control to the list
void kb_controls_add_control(kb_controls* list, kb_type type, void* ptr);

// Input processing
bool kb_process_input(kb_controls* list, uint8_t button, int x, int y);
void kb_process_mouse_motion(kb_controls* list, uint8_t button, int x, int y, int x_rel, int y_rel);

/* ******************************************************************* */
/* Button */
typedef void (*kb_button_cb)(void *placeholder);
 
#pragma pack(push)
typedef struct
{
	uint32_t id;
	SDL_Surface* btn_norm;
	SDL_Surface* btn_hover;
	//SDL_Surface* btn_pressed;
	SDL_Rect box;
	kb_button_cb callback;
	uint8_t state;
} kb_button;
#pragma pack(pop)

// Create Button
kb_button* kb_button_create(kb_controls* list, int width, int height, int xpos, int ypos, kb_button_cb button_pressed_callback);

// Destroy Button
void kb_button_destroy(kb_controls* list, kb_button* btn);

/* ******************************************************************* */
/* Slider */
typedef void (*kb_slider_cb)(float slider_val);
 
#pragma pack(push)
typedef struct
{
	uint32_t id;
	SDL_Surface* slider_knob_norm;
	SDL_Surface* slider_knob_hover;
	SDL_Surface* slider_pane;
	SDL_Rect box;
	SDL_Rect pane_box;
	SDL_Rect knob_box;
	kb_slider_cb callback;
	uint8_t state;
} kb_slider;
#pragma pack(pop)

// Create Slider
kb_slider* kb_slider_create(kb_controls* list, int width, int height, int xpos, int ypos, kb_slider_cb slider_moved_callback, float initial_value);

// Destroy Slider
void kb_slider_destroy(kb_controls* list, kb_slider* slider);

#endif
