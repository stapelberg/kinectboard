// vim:ts=4:sw=4:expandtab

#include "kinectboard_controls.h"
#include <malloc.h>

static uint32_t CONTROL_ID = 0;

/* ******************************************************************* */
/* KB Utility functions */
int __kb_mouse_over(int mouse_x, int mouse_y, SDL_Rect* target) {
     if (( mouse_x > target->x )
                  && ( mouse_x < target->x + target->w )
                  && ( mouse_y > target->y )
                  && ( mouse_y < target->y + target->h )) {
              return 1;
          }
    return 0;
}

SDL_Surface* __kb_surface_fill_color(SDL_Rect* box, SDL_Color* color) {
    Uint32 rmask, gmask, bmask; //, amask;
      #if SDL_BYTEORDER == SDL_BIG_ENDIAN
         rmask = 0xff000000;
         gmask = 0x00ff0000;
         bmask = 0x0000ff00;
       //  amask = 0x000000ff;
      #else
         rmask = 0x000000ff;
         gmask = 0x0000ff00;
         bmask = 0x00ff0000;
       //  amask = 0xff000000;
      #endif
      
    SDL_Surface* surface = SDL_CreateRGBSurface(SDL_SWSURFACE, box->w, box->h, 24/*32*/, rmask, gmask, bmask, /*amask*/0);
    SDL_LockSurface(surface);
    SDL_FillRect(surface, box, SDL_MapRGB(surface->format, color->r, color->g, color->b));
    SDL_UnlockSurface(surface);
    
    return surface;
}

int __compare_btn(kb_button* lhs, kb_button* rhs) {
    return (lhs != 0 && rhs != 0) ? (rhs->id == lhs->id ? 0 : 1) : -1; 
}

void __kb_unregister_button(kb_controls* list, kb_button* btn) {
    if(!list) { return; }
    if(list->root->type == KB_BUTTON) {
        if(__compare_btn((kb_button*)list->root, btn) == 0) {
            list->root = list->root->next;
        } 
    }
    
    kb_controls_node* ptr = list->root;	
	while(ptr != 0) {
        if(ptr->type == KB_BUTTON && ptr->data != 0 && __compare_btn((kb_button*)ptr->data, btn) == 0) {
            ptr->data = ptr->next;
        }
    }
}

/* ******************************************************************* */
/* KB Button */
kb_button* kb_button_create(kb_controls* list, int width, int height, int xpos, int ypos, kb_button_cb button_pressed_callback)
{
	kb_button* btn = (kb_button*)malloc(sizeof(kb_button));
    
	btn->callback = button_pressed_callback;

    btn->box.w = width;
	btn->box.h = height;
    
    // Apply offsets later
    btn->box.x = 0;
    btn->box.y = 0;
    
    // Add some color
    SDL_Color color_normal = {255,220,100};
    SDL_Color color_hover = {255,240,100};

	btn->btn_norm  = __kb_surface_fill_color(&btn->box, &color_normal);
    btn->btn_hover = __kb_surface_fill_color(&btn->box, &color_hover);
    
    // apply offset
    btn->box.x = xpos;
	btn->box.y = ypos;

    btn->state = 0;
    btn->id = ++CONTROL_ID;
	return btn;
}

void kb_button_destroy(kb_controls* list, kb_button* btn) 
{
	__kb_unregister_button(list, btn);
	SDL_FreeSurface(btn->btn_norm);
	SDL_FreeSurface(btn->btn_hover);
	
	free(btn);
	btn = 0;
}
/* ******************************************************************* */
/* KB Controls Logic */

kb_controls* kb_controls_create() 
{
	kb_controls* list = (kb_controls*)malloc(sizeof(kb_controls));
	list->root = 0;
	list->size = 0;
	return list; 
}

void kb_controls_destroy(kb_controls* ptr)
{
	free(ptr);
}

void kb_controls_add_control(kb_controls* list, kb_type type, void* ptr)
{
	kb_controls_node* node = (kb_controls_node*)malloc(sizeof(kb_controls_node));
	node->parent = 0;
	node->next = 0;
	node->data = ptr;
	node->type = type;
	
	if(!list) { 
		free(node); 
		return; 
	}
	
	if(!list->root && list->size == 0)  {
		list->size++;
		list->root = node;
		return;
	}
    
    kb_controls_node* next = list->root; 
    
    while(next->next != 0) {
        next = next->next;
    }
    
    
    next->next = node;
    node->parent = next;
    list->size++;
}

void kb_process_mouse_motion(kb_controls* list, int x, int y)
{    
    if(!list) { return; } 

	kb_controls_node* ptr = list->root;
		
	while(ptr != 0) {
        if(ptr->data != 0) {
            if(ptr->type == KB_BUTTON) {
                kb_button* btn = (kb_button*)ptr->data;
                btn->state = __kb_mouse_over(x, y, &btn->box); 
            }
        }
        ptr = ptr->next;
	}
}

void kb_process_input(kb_controls* list, uint8_t button, int x, int y)
{    
    if(!list) { return; } 

    kb_controls_node* ptr = list->root;	
	while(ptr != 0) {
        if(ptr->data != 0) {
            if(ptr->type == KB_BUTTON) {
                kb_button* btn = (kb_button*)ptr->data;
                if(__kb_mouse_over(x, y, &btn->box) > 0) {
                    btn->callback(0);
                }
            }
        }
		
        ptr = ptr->next;
	}
}
 
void kb_controls_render(kb_controls* list, SDL_Surface* screen) 
{
	if(!list) { return; } 

	kb_controls_node* ptr = list->root;
	while(ptr != 0) {
        if(ptr->data != 0) {
            if(ptr->type == KB_BUTTON) {
                kb_button* btn = (kb_button*)ptr->data;
                SDL_BlitSurface(btn->state > 0 ?  btn->btn_hover : btn->btn_norm, NULL, screen, &btn->box);
            }
        }
		
        ptr = ptr->next;
	}
}
