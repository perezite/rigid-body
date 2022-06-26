#include <stdio.h>
#include <windows.h>
#include <gl\glut.h>
//#include <gl\glaux.h>
#include <gl\glu.h>
#include "Physics.h"

// G L O B A L S================================================================================

Simulator World(800, 600);
OBJECT Temp;

const int WorldWidth = 800;
const int WorldHeight = 600;

bool GravityActive = true;
vector_2 Gravity(0, -10.0f);                // Fallbeschleunigung

vector_2 BlackHolePos1(0.0f, 0.0f);         // Schwarzes Loch 1
vector_2 BlackHolePos2(2.0f, 2.0f);         // Schwarzes Loch 2

bool Spring1Active = true;
bool Spring2Active = false;
float const Kws = 20.0f;                    // Hook'sche Federkonstante
vector_2 AnchorPos1(0, 0);                  // Federhalterung 1
vector_2 AnchorPos2(2, 0);                  // Federhalterung 2

bool DampingActive = true;
float LinearDampingFactor = 1.0f;
float AngularDampingFactor = 1.5f;

bool GroundActive = true;
float GroundPlane = -3.0;

int SourceConfigurationIndex = 0;
int TargetConfigurationIndex = 1;

const GLfloat ZPlane = -10.0f;              // 2D-Zeichenebene


// F U N C T I O N S============================================================================

/*---------------------------------------------------------------------
// Ein Object initialisieren
---------------------------------------------------------------------*/

void InitObject(OBJECT &Object, float Width, float Height, float Mass)
{

    Object.type = 0;                       // Temporär: 0 == Viereck
    Object.Width = Width;
    Object.Height = Height;
    Object.OneOverMass = 1/Mass;
    Object.OneOverCMMomentOfIntertia = 1/((Mass/12)*(Width*Width+Height*Height)); // Für Viereck
    Object.Configurations[0].Orientation = 0;
    Object.Configurations[0].AngularVelocity = 0;
    Object.Configurations[0].Torque = 0;
    Object.Configurations[0].CMVelocity = vector_2(0, 0);
    Object.Configurations[0].CMForce = vector_2(0, 0);
    // Bounding-Box des Vierecks
    Object.Configurations[0].BoundingBox.Vertices[0] = vector_2(1.0, -1.5);
    Object.Configurations[0].BoundingBox.Vertices[1] = vector_2(Width + 1.0, -1.5);
    Object.Configurations[0].BoundingBox.Vertices[2] = vector_2(Width + 1.0, Height - 1.5);
    Object.Configurations[0].BoundingBox.Vertices[3] = vector_2(0 + 1.0, Height - 1.5);
    // Die CM-Position ausrechnen (Temporär für Viereck)
    vector_2 CMPos;
    for (int i = 0; i < 4; i++) // Alle Vertex-Ortsvektoren mit ihrer Masse aufaddieren
    {
        CMPos = CMPos + Object.Configurations[0].BoundingBox.Vertices[i]
                * 0.25 * (1/Object.OneOverMass);
    }
    CMPos = CMPos * Object.OneOverMass; // Vektorsumme durch Gesamtmasse dividieren
    Object.Configurations[0].CMPosition = CMPos;

}

/*---------------------------------------------------------------------
// Simulator - Konstruktor
---------------------------------------------------------------------*/

Simulator::Simulator(float WorldWidth, float WorldHeight)
{
    InitObject(Temp, 0.5, 0.5, 5);
}

/*---------------------------------------------------------------------
// Simulator - Destruktor
---------------------------------------------------------------------*/

Simulator::~Simulator(void)
{

}

/*---------------------------------------------------------------------
// Hauptfunktion für den Simulator: Über den Zeitraum DeltaTime die Bewegungen simulieren
---------------------------------------------------------------------*/

void Simulator::Simulate(float DeltaTime)
{

    ComputeForces(SourceConfigurationIndex);
    Integrate(DeltaTime);
    CalculateVertices(TargetConfigurationIndex);
    if (GroundActive)
        CheckForCollision();

    SourceConfigurationIndex = TargetConfigurationIndex;
    TargetConfigurationIndex = SourceConfigurationIndex;
}

/*---------------------------------------------------------------------
// Kräfte berechnen, die im Moment auf die Objekte wirken
---------------------------------------------------------------------*/

void Simulator::ComputeForces(int ConfigurationIndex)
{
    // Objekt-Konfiguration und Box holen
    OBJECT::configuration &Configuration = Temp.Configurations[ConfigurationIndex];
    OBJECT::configuration::bounding_box &Box = Temp.Configurations[ConfigurationIndex].BoundingBox;

    // Kraft und Drehmoment von letztem Frame löschen
    Configuration.Torque = 0;
    Configuration.CMForce = vector_2(0,0);


    // Gravitation
    if (GravityActive)
    {
        Configuration.CMForce += (1/Temp.OneOverMass)*Gravity;
    }

    // "Schwarzes Loch" 1
    if (GetAsyncKeyState('A'))
    {
        vector_2 Force;
        Force = BlackHolePos1 - Configuration.CMPosition;
        Force = Force * 5 * (1/Temp.OneOverMass);

        Configuration.CMForce += Force;
    }

    // "Schwarzes Loch" 2
    if (GetAsyncKeyState('S'))
    {
        vector_2 Force2;
        Force2 = BlackHolePos2 - Configuration.CMPosition;
        Force2 = Force2 * 10;

        Configuration.CMForce += Force2;
    }

    // Feder 1
    if (Spring1Active)
    {
        // Federkraft
        vector_2 Spring = -Kws * (Box.Vertices[3] - AnchorPos1);

        // Federkraft auf CM updaten
        Configuration.CMForce += Spring;

        // Abstand von CM zu Angriffspunkt der Kraft
        vector_2 R = Box.Vertices[3] - Temp.Configurations[ConfigurationIndex].CMPosition;

        // Drehmoment updaten
        Configuration.Torque += PerpDotProduct(R, Spring);  // R = PerpVektor
    }

    // Feder 2
    if (Spring2Active)
    {
        // Federkraft
        vector_2 Spring = -Kws * (Box.Vertices[2] - AnchorPos2);

        // Federkraft auf CM updaten
        Configuration.CMForce += Spring;

        // Abstand von CM zu Angriffspunkt der Kraft
        vector_2 R = Box.Vertices[2] -
                     Temp.Configurations[ConfigurationIndex].CMPosition;

        // Drehmoment updaten
        Configuration.Torque += PerpDotProduct(R, Spring);  // R = PerpVektor

    }

    // Dämpfung
    if (DampingActive)
    {
        Configuration.Torque  += -AngularDampingFactor * Configuration.AngularVelocity;
        Configuration.CMForce += -LinearDampingFactor * Configuration.CMVelocity;
    }

    // Tastatureingaben

    if (GetAsyncKeyState('G'))
    {
        GravityActive = !GravityActive;
        Sleep(200);
    }

    if (GetAsyncKeyState('Y'))
    {
        Spring1Active = !Spring1Active;
        Sleep(200);
    }

    if (GetAsyncKeyState('X'))
    {
        Spring2Active = !Spring2Active;
        Sleep(200);
    }

    if (GetAsyncKeyState(VK_LEFT))
    {
        AnchorPos1 += vector_2(-0.1, 0.0);
    }

    if (GetAsyncKeyState(VK_RIGHT))
    {
        AnchorPos1 += vector_2(0.1, 0.0);
    }

    if (GetAsyncKeyState(VK_UP))
    {
        AnchorPos1 += vector_2(0.0, 0.1);
    }

    if (GetAsyncKeyState(VK_DOWN))
    {
        AnchorPos1 += vector_2(0.0, -0.1);
    }

}

/*---------------------------------------------------------------------
// Source-Konfiguration integrieren um Target-Konfiguration zu erhalten
---------------------------------------------------------------------*/

void Simulator::Integrate(float DeltaTime)
{
    // Objekt-Source und Target-Konfigurationen holen
    OBJECT::configuration &Source = Temp.Configurations[SourceConfigurationIndex];
    OBJECT::configuration &Target = Temp.Configurations[TargetConfigurationIndex];


    // Position integrieren
    Target.CMPosition = Source.CMPosition + DeltaTime * Source.CMVelocity;

    // Lineargeschw. integrieren
    Target.CMVelocity = Source.CMVelocity + DeltaTime * (Temp.OneOverMass * Source.CMForce);

    // Orientierung integrieren
    Target.Orientation = Source.Orientation + DeltaTime * Source.AngularVelocity;

    // Winkelgeschw. integrieren
    Target.AngularVelocity = Source.AngularVelocity + DeltaTime *
                             (Temp.OneOverCMMomentOfIntertia * Source.Torque);
}

/*---------------------------------------------------------------------
// Vertex-Positionen updaten
---------------------------------------------------------------------*/

void Simulator::CalculateVertices(int ConfigurationIndex)
{
    // Entfernung zur CM-Position in X-/Y-Richtung (Für Vierecke)
    float const Width = Temp.Width / 2;
    float const Height = Temp.Height / 2;

    // Rotationsmatrix aus Orientierung berechnen
    matrix_2x2 Rotation(Temp.Configurations[ConfigurationIndex].Orientation);

    // Bounding-Box holen
    OBJECT::configuration::bounding_box &Box =
        Temp.Configurations[ConfigurationIndex].BoundingBox;

    // CM-Position holen
    vector_2 &CMPos = Temp.Configurations[ConfigurationIndex].CMPosition;

    // Vertices updaten
    vector_2 Position = Temp.Configurations[ConfigurationIndex].CMPosition;
    Box.Vertices[0] = Position + Rotation * vector_2(-Width, -Height);
    Box.Vertices[1] = Position + Rotation * vector_2( Width, -Height);
    Box.Vertices[2] = Position + Rotation * vector_2( Width,  Height);
    Box.Vertices[3] = Position + Rotation * vector_2(-Width,  Height);

}

/*---------------------------------------------------------------------
// Simulator-Szene zeichnen
---------------------------------------------------------------------*/

void Simulator::Render(void)
{
    int TCI = TargetConfigurationIndex;

    glLoadIdentity();

    // Koords-Ursprung mit Kugel markieren
    glTranslatef(0.0f, 0.0f, ZPlane);
    glutSolidSphere(0.01f, 20, 20);
    glLoadIdentity();

    // Kiste malen
    glBegin(GL_LINE_LOOP);
    glVertex3f(Temp.Configurations[TCI].BoundingBox.Vertices[0].X, Temp.Configurations[TCI].BoundingBox.Vertices[0].Y, ZPlane);
    glVertex3f(Temp.Configurations[TCI].BoundingBox.Vertices[1].X, Temp.Configurations[TCI].BoundingBox.Vertices[1].Y, ZPlane);
    glVertex3f(Temp.Configurations[TCI].BoundingBox.Vertices[2].X, Temp.Configurations[TCI].BoundingBox.Vertices[2].Y, ZPlane);
    glVertex3f(Temp.Configurations[TCI].BoundingBox.Vertices[3].X, Temp.Configurations[TCI].BoundingBox.Vertices[3].Y, ZPlane);
    glEnd();


    // Welt-Feder 1 malen
    if (Spring1Active)
    {
        glBegin(GL_LINES);
        glVertex3f(AnchorPos1.X, AnchorPos1.Y, ZPlane);
        glVertex3f(Temp.Configurations[TCI].BoundingBox.Vertices[3].X, Temp.Configurations[TCI].BoundingBox.Vertices[3].Y, ZPlane);
        glEnd();
    }
    // Welt-Feder 2 malen
    if (Spring2Active)
    {
        glBegin(GL_LINES);
        glVertex3f(AnchorPos2.X, AnchorPos2.Y, ZPlane);
        glVertex3f(Temp.Configurations[TCI].BoundingBox.Vertices[2].X, Temp.Configurations[TCI].BoundingBox.Vertices[2].Y, ZPlane);
        glEnd();
    }

    // Boden malen
    if (GroundActive)
    {
        glBegin(GL_LINES);
        glVertex3f(-10.0f, GroundPlane, ZPlane);
        glVertex3f( 10.0f, GroundPlane, ZPlane);
        glEnd();
    }

    glBegin(GL_LINES);
    glVertex3f(1.0f, 1.0f, -101.2f);
    glVertex3f(1.0f, -1.0f, -101.2f);
    glEnd();

}

/*---------------------------------------------------------------------
// Temp
---------------------------------------------------------------------*/

void AddImpulse(int Corner)
{
    // Es wird angenommen, dass die Masse und das Trägheitsmoment des zweiten
    // Objekts gegen unendlich gehen
    // d.h. alle Terme mit .../Mb und .../Ib gehen gegen 0

    float CoefficientOfRestitution = 0.4f;
    OBJECT::configuration &Configuration = Temp.Configurations[TargetConfigurationIndex];

    vector_2 CollisionNormal = vector_2(0.0f, -1.0f);

    vector_2 Position = Configuration.BoundingBox.Vertices[Corner];
    vector_2 CMToCornerPerp = GetPerpendicular(Position - Configuration.CMPosition);
    vector_2 Velocity = Configuration.CMVelocity + Configuration.AngularVelocity * CMToCornerPerp;

    float ImpulseNumerator = -(float(1) - CoefficientOfRestitution) * DotProduct(Velocity, CollisionNormal);
    float PerpDot = DotProduct(CMToCornerPerp, CollisionNormal);
    float ImpulseDenominator = Temp.OneOverMass + Temp.OneOverCMMomentOfIntertia * PerpDot * PerpDot;
    float Impulse = ImpulseNumerator/ImpulseDenominator;


    Configuration.CMVelocity += Impulse * Temp.OneOverMass * CollisionNormal;
    Configuration.AngularVelocity += Impulse * Temp.OneOverCMMomentOfIntertia * PerpDot;
}

/*---------------------------------------------------------------------
// Temp
---------------------------------------------------------------------*/

void Simulator::CheckForCollision(void)
{
    OBJECT::configuration &Configuration = Temp.Configurations[TargetConfigurationIndex];
    float MaxPenetration = 0.0f; // Boden-Eindringtiefe des am weitesten eingedrungenen Vertex

    for (int i = 0; i < 4; i++)
    {
        if (Configuration.BoundingBox.Vertices[i].Y < GroundPlane)
        {
            // Eindringtiefe des Vertex in den Boden berechnen
            float Penetration = fabs(Configuration.BoundingBox.Vertices[i].Y - GroundPlane);
            if (Penetration > MaxPenetration)
            {
                MaxPenetration = Penetration;
            }

            // Impuls bezgl. des Kollisions-Vertex zum Objekt hinzufügen
            AddImpulse(i);
        }
    }

    // Kiste direkt zum Boden verschieben
    Configuration.CMPosition += vector_2(0.0f, MaxPenetration);
    for (int i = 0; i < 4; i++)
        Configuration.BoundingBox.Vertices[i] += vector_2(0.0f, MaxPenetration);

}



void PassiveMotionFunc(int x, int y)
{

    // Manuelle Berechnung (Veraltet)
    /*    y = -y;         // Koord.-System umdrehen
        float fX = ((float)x)/100 - 3.0f;
        float fY = ((float)y)/171 + 0.2f;
        AnchorPos1 = vector_2(fX, fY); */

    // Berechnung mit Glu

    GLdouble Modelview[16], Project[16];
    GLint Viewport[4];
    GLdouble objX, objY, objZ;
    glGetDoublev(GL_MODELVIEW_MATRIX, Modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, Project);
    glGetIntegerv(GL_VIEWPORT, Viewport);
    GLdouble winz = ((-ZPlane) - 1.0f)/(100.0f);
    gluUnProject(x, y, 0.0f, Modelview, Project, Viewport, &objX, &objY, &objZ);  // 0.99f
    AnchorPos1 = vector_2(objX, -objY);
}
