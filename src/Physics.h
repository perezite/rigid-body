#ifndef __PHYSICS_H_
#define __PHYSICS_H_

#include "Math.h"

// K L A S S E N============================================================================

typedef struct
{
    char type;
    float Width, Height;
    float OneOverMass, OneOverCMMomentOfIntertia;
    enum {NumberOfConfigurations = 2};

    struct configuration
    {
        vector_2 CMPosition;
        float Orientation;
        vector_2 CMVelocity;
        float AngularVelocity;
        vector_2 CMForce;
        float Torque;

        struct bounding_box
        {
            vector_2 Vertices[4];
        }BoundingBox;

    } Configurations[NumberOfConfigurations];
}OBJECT, *LPOBJECT;


class Simulator
{
public:

    Simulator(float WorldWidth, float WorldHeight);
    void Simulate(float DeltaTime);
    void Render(void);
    ~Simulator(void);

private:

    float WorldWidth, WorldHeight;

    enum collision_state{Penetrating, Colliding, Clear};
    vector_2 CollisionNormal;

    void ComputeForces(int ConfigurationIndex);
    void Integrate(float DeltaTime);
    void CalculateVertices(int ConfigurationIndex);
//    collision_state CheckForCollisions(int ConfigurationIndex);
//    void ResolveCollisions(int ConfigurationIndex);
    void CheckForCollision(void);

};

// P R O T O S==============================================================================

void InitObject(OBJECT &Object, float Width, float Height, float Mass);
void PassiveMotionFunc(int x, int y);


#endif
