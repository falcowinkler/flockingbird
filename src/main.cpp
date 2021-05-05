#include <cairo.h>
#include <gtkmm.h>
#include <gtkmm/drawingarea.h>
#include <gtkmm/window.h>
#include <iostream>

class Point : public Gtk::DrawingArea {
public:
    Point(){};
};

int main(int argc, char* argv[]) {
    Gtk::Main   kit(argc, argv);
    Gtk::Window window;
    window.set_title("Libgnomedbmm example window");
    window.set_default_size(400, 400);
    window.show_all();
    kit.run(window);
    return 0;
}
