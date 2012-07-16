// vim:ts=4:sw=4:expandtab
/*
 * Kinect Board Controls 
 */

#ifndef __KINECTBOARD_CONTROLS
#define __KINECTBOARD_CONTROLS
 
 #include <sys/types.h>
 #include <SDL/SDL.h>
 #include <SDL/SDL_ttf.h>
 #include <stdbool.h>
 
#ifdef __cplusplus
extern "C" {
#endif

/* Inits the UI */ 
void kb_ui_init();

/* Update the UI */
void kb_ui_update();

/* Render the UI */ 
void kb_ui_render();

/* Destroy the UI */ 
void kb_ui_destroy();

/* inject mouse move */
void kb_ui_inject_mouse(Uint16 x,Uint16 y);

/* inject mouse button */
void kb_ui_inject_mouse_button(uint8_t button, bool down);

/* Call a Javascript function with an array of args. */
void kb_ui_call_javascript(const char* function, char* argv);

typedef void (*kb_ui_js_value_callback)(float set_val);
typedef void (*kb_ui_js_void_callback)(void);

/* Register a js callback */
void kb_ui_register_void_callback(const char* js_function_match, kb_ui_js_void_callback callback);
void kb_ui_register_value_callback(const char* js_function_match, kb_ui_js_value_callback callback);

#ifdef __cplusplus
}
#endif

#endif