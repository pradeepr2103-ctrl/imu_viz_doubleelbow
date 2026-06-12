#include "renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glut.h>

void Renderer::initialize()
{
    int argc = 0;
    glutInit(&argc, nullptr);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    setupLighting();
    humanModel.load("human.glb");
}

void Renderer::setupLighting()
{
    GLfloat amb[]  = {0.30f, 0.30f, 0.30f, 1.0f};
    GLfloat diff[] = {0.90f, 0.88f, 0.84f, 1.0f};
    GLfloat spec[] = {0.30f, 0.30f, 0.30f, 1.0f};
    GLfloat pos0[] = {8.0f, 18.0f, 10.0f, 1.0f};
    GLfloat pos1[] = {-5.0f, 10.0f, -6.0f, 1.0f};
    GLfloat fill[] = {0.25f, 0.25f, 0.28f, 1.0f};
    GLfloat zero[] = {0.0f,  0.0f,  0.0f, 1.0f};

    glLightfv(GL_LIGHT0, GL_POSITION, pos0);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);
    glLightfv(GL_LIGHT1, GL_POSITION, pos1);
    glLightfv(GL_LIGHT1, GL_AMBIENT,  zero);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  fill);
    glLightfv(GL_LIGHT1, GL_SPECULAR, zero);
}

void Renderer::cycleCameraView()
{
    switch (cameraView) {
        case CameraView::FRONT: cameraView = CameraView::BACK;  break;
        case CameraView::BACK:  cameraView = CameraView::SIDE;  break;
        case CameraView::SIDE:  cameraView = CameraView::FRONT; break;
    }
}

void Renderer::render(
    const glm::quat& leftForearmQ,
    const glm::quat& rightForearmQ,
    const glm::quat& leftUpperArmQ,
    const glm::quat& rightUpperArmQ,
    const glm::quat& leftThighQ,
    const glm::quat& rightThighQ,
    const glm::quat& leftShinQ,
    const glm::quat& rightShinQ,
    const glm::quat& hipsQ,
    const glm::quat& chestQ)
{
    glClearColor(0.07f, 0.07f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT;
    glFrustum(-aspect * 0.5f, aspect * 0.5f, -0.5f, 0.5f, 1.5f, 200.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    switch (cameraView) {
        case CameraView::FRONT:
            glTranslatef(0.0f, -9.0f, -38.0f);
            break;
        case CameraView::BACK:
            glTranslatef(0.0f, -9.0f, -38.0f);
            glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
            break;
        case CameraView::SIDE:
            glTranslatef(0.0f, -9.0f, -38.0f);
            glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
            break;
    }

    setupLighting();
    humanModel.draw(
        leftForearmQ, rightForearmQ,
        leftUpperArmQ, rightUpperArmQ,
        leftThighQ, rightThighQ,
        leftShinQ, rightShinQ,
        hipsQ, chestQ
    );

    drawWorldAxes();

    drawTrackingAxesHud(
        leftForearmQ, rightForearmQ,
        leftUpperArmQ, rightUpperArmQ,
        leftThighQ, rightThighQ,
        leftShinQ, rightShinQ,
        hipsQ, chestQ
    );
}

void Renderer::drawWorldAxes()
{
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(3,0,0);
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,3,0);
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,3);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::drawTrackingAxesHud(const glm::quat& lFAQ,
                                   const glm::quat& rFAQ,
                                   const glm::quat& lUAQ,
                                   const glm::quat& rUAQ,
                                   const glm::quat& lTHQ,
                                   const glm::quat& rTHQ,
                                   const glm::quat& lSHQ,
                                   const glm::quat& rSHQ,
                                   const glm::quat& hipsQ,
                                   const glm::quat& chestQ)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, WINDOW_WIDTH, 0.0, WINDOW_HEIGHT, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    const float scale = 18.0f;
    const float gap   = 60.0f;
    const float x     = (float)WINDOW_WIDTH - 70.0f;
    const float x2    = x - 80.0f;
    const float x3    = x2 - 80.0f;
    const float top   = (float)WINDOW_HEIGHT - 50.0f;

    auto drawLabel = [&](float px, float py, const char* text) {
        glColor3f(0.90f, 0.90f, 0.92f);
        glRasterPos2f(px - 14.0f, py + 24.0f);
        for (const char* c = text; *c; ++c)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    };

    drawLabel(x,  top,              "L_FA");  drawHudAxisWidget(x,  top,              lFAQ,  scale);
    drawLabel(x,  top - gap,        "R_FA");  drawHudAxisWidget(x,  top - gap,        rFAQ,  scale);
    drawLabel(x,  top - gap * 2.0f, "L_UA");  drawHudAxisWidget(x,  top - gap * 2.0f, lUAQ,  scale);
    drawLabel(x,  top - gap * 3.0f, "R_UA");  drawHudAxisWidget(x,  top - gap * 3.0f, rUAQ,  scale);
    drawLabel(x2, top,              "L_TH");  drawHudAxisWidget(x2, top,              lTHQ,  scale);
    drawLabel(x2, top - gap,        "R_TH");  drawHudAxisWidget(x2, top - gap,        rTHQ,  scale);
    drawLabel(x2, top - gap * 2.0f, "L_SH");  drawHudAxisWidget(x2, top - gap * 2.0f, lSHQ,  scale);
    drawLabel(x2, top - gap * 3.0f, "R_SH");  drawHudAxisWidget(x2, top - gap * 3.0f, rSHQ,  scale);
    drawLabel(x3, top,              "HIPS");  drawHudAxisWidget(x3, top,              hipsQ, scale);
    drawLabel(x3, top - gap,        "CHEST"); drawHudAxisWidget(x3, top - gap,        chestQ, scale);

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void Renderer::drawHudAxisWidget(float cx, float cy,
                                 const glm::quat& q,
                                 float scale)
{
    glm::mat4 viewRot(1.0f);
    switch (cameraView) {
        case CameraView::FRONT:
            break;
        case CameraView::BACK:
            viewRot = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0,1,0));
            break;
        case CameraView::SIDE:
            viewRot = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0,1,0));
            break;
    }

    glm::vec3 axes[3] = {
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(1,0,0), 0.0f)),
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(0,1,0), 0.0f)),
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(0,0,1), 0.0f))
    };

    glColor3f(0.22f, 0.22f, 0.26f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(cx - 24.0f, cy - 20.0f);
    glVertex2f(cx + 24.0f, cy - 20.0f);
    glVertex2f(cx + 24.0f, cy + 20.0f);
    glVertex2f(cx - 24.0f, cy + 20.0f);
    glEnd();

    glPointSize(4.0f);
    glColor3f(0.85f, 0.85f, 0.88f);
    glBegin(GL_POINTS);
    glVertex2f(cx, cy);
    glEnd();

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.12f, 0.10f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[0].x * scale, cy + axes[0].y * scale);

    glColor3f(0.20f, 1.0f, 0.20f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[1].x * scale, cy + axes[1].y * scale);

    glColor3f(0.20f, 0.45f, 1.0f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[2].x * scale, cy + axes[2].y * scale);
    glEnd();
    glLineWidth(1.0f);
    glPointSize(1.0f);
}