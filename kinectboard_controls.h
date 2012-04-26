// vim:ts=4:sw=4:expandtab
/*
 * Kinect Board Controls 
 */

#ifndef __KINECTBOARD_CONTROLS
#define __KINECTBOARD_CONTROLS
 
 #include <sys/types.h>
 #include <SDL/SDL.h>
 
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
void kb_process_input(kb_controls* list, uint8_t button, int x, int y);
void kb_process_mouse_motion(kb_controls* list, int x, int y);

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

#endif