// RayCasterDriver.cpp
// -- front end for ray casting assignment
// cs500 9/19

#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include "SDL2/SDL.h"
#include "GL/glew.h"
#include <GL/gl.h>
#include "RayCaster.h"
using namespace std;


class Client {
  public:
    Client(SDL_Window *w);
    ~Client(void);
    void load(const char *fname);
    void renderStart(void);
    void renderStop(void);
    void resize(void);
    void blit(void);
    void save(void);
  private:
    Bitmap *bitmap;
    Scene scene;
    thread *render_thread;
    bool kill_flag;
    SDL_Window *window;
    static void draw(Client &client);
};


Client::Client(SDL_Window *w)
    : render_thread(nullptr),
      kill_flag(false),
      window(w) {
  bitmap = new Bitmap(500,500);
  for (int i=0; i < bitmap->height(); ++i)
    for (int j=0; j < bitmap->width(); ++j)
      bitmap->setPixel(i,j,100,0,100);
}


Client::~Client(void) {
  renderStop();
  clearScene(scene);
  delete bitmap;
}


void Client::load(const char *fname) {
  clearScene(scene);
  try {
    loadFile(fname,scene);
  }
  catch (exception &e) {
    cout << e.what() << endl;
  }
}


void Client::draw(Client &client) {
  SDL_SetWindowTitle(client.window,"rendering");
  renderScene(client.scene,*client.bitmap,&client.kill_flag);
  SDL_SetWindowTitle(client.window,"finished rendering");
}


void Client::renderStart(void) {
  kill_flag = false;
  render_thread = new thread(draw,ref(*this));
}


void Client::renderStop(void) {
  if (render_thread != nullptr) {
    kill_flag = true;
    render_thread->join();
    delete render_thread;
    render_thread = nullptr;
  }
}


void Client::blit(void) {
  glDrawPixels(bitmap->width(),bitmap->height(),GL_RGB,
               GL_UNSIGNED_BYTE,bitmap->bytes());
  SDL_GL_SwapWindow(window);
}


void Client::resize(void) {
  int W, H;
  SDL_GetWindowSize(window,&W,&H);
  float aspect = glm::length(scene.camera.right)/glm::length(scene.camera.up);
  W = round(H*aspect);
  SDL_SetWindowSize(window,W,H);
  delete bitmap;
  bitmap = new Bitmap(W,H);
}


void Client::save(void) {
  // write bitmap file
  int data_size = bitmap->height()*bitmap->stride();
  fstream out("RayCastTest.bmp",ios_base::binary|ios_base::out);
  char header[54] = { 'B', 'M', 0 };
  *reinterpret_cast<unsigned*>(header+2) = 54+data_size;
  *reinterpret_cast<unsigned*>(header+10) = 54;
  *reinterpret_cast<unsigned*>(header+14) = 40;
  *reinterpret_cast<int*>(header+18) = bitmap->width();
  *reinterpret_cast<int*>(header+22) = bitmap->height();
  *reinterpret_cast<unsigned short*>(header+26) = 1;
  *reinterpret_cast<unsigned short*>(header+28) = 24;
  *reinterpret_cast<unsigned*>(header+34) = data_size;
  out.write(header,54);
  char *data = new char[data_size];
  for (int j=0; j < bitmap->height(); ++j)
    for (int i=0; i < bitmap->width(); ++i) {
      int index = j*bitmap->stride() + i*3;
      // write in BGR order
      data[index+0] = bitmap->bytes()[index+2];
      data[index+1] = bitmap->bytes()[index+1];
      data[index+2] = bitmap->bytes()[index+0];
    }
  out.write(data,data_size);
  delete[] data;
}


/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  // SDL: initialize and create a window
  SDL_Init(SDL_INIT_VIDEO);
  const char *title = "Press 'p' to write bitmap to file";
  int width = 500,
      height = 500;
  SDL_Window *window = SDL_CreateWindow(title,SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED,width,height,
                                        SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
  SDL_GLContext context = SDL_GL_CreateContext(window);

  Client *client = new Client(window);

  bool done = false;

  while (!done) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
        case SDL_QUIT:
          done = true;
          break;
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_ESCAPE)
            done = true;
          else if (event.key.keysym.sym == SDLK_p)
            client->save();
          break;
        case SDL_DROPFILE:
          client->renderStop();
          client->load(event.drop.file);
          client->resize();
          client->renderStart();
          SDL_free(event.drop.file);
          break;
      }
    }
    client->blit();
  }

  // clean up
  delete client;
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}

