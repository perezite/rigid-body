#include <windows.h>
#include <stdio.h>
#include <gl\glut.h>
//#include <gl\glaux.h>
#include "Physics.h"

Simulator simul(800, 600);

void instructions(void)
{
    char instruction[] = "Steuerung:\n\nPfeiltasten: Position des linken Seils ändern\nY: Linkes Seil aktivieren/deaktivieren\nX: Rechtes Seil aktivieren/deaktivieren\nG: Gravitation aktivieren/deaktivieren\nA: Gravitationsfeld im Bildzentrum erzeugen\nS: Gravitationsfeld rechts oberhalb des Zentrums erzeugen\n";
    MessageBox(NULL, instruction, "Steuerung", NULL);

}


void display(void)
{
    static DWORD FrameTime;
    DWORD StartTime = GetTickCount();

    glClear (GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    simul.Simulate(((float)FrameTime)/1000);              // Durch 1000, damit Sekunden
    simul.Render();

    glutSwapBuffers();
    glutPostRedisplay();

    while ((GetTickCount() - StartTime)<14)
    {
        Sleep(1);    // Framebremse auf 75 fps
    }
    FrameTime = GetTickCount() - StartTime;             // Frametime updaten

}

void init (void)
{

    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize (800, 600);
    glutInitWindowPosition (100, 100);
    glutCreateWindow ("2D Rigid Body");

    glClearColor (0.0, 0.0, 0.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f,(GLfloat)800/(GLfloat)600,10.0f,100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    GLfloat test;

    instructions();

}


int main(int argc, char** argv)
{

    glutInit(&argc, argv);
    init();
    glutDisplayFunc(display);
    glutPassiveMotionFunc(PassiveMotionFunc);
    glutMainLoop();
    return 0;

}
