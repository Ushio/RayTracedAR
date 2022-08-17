#include "pr.hpp"
#include <iostream>
#include <memory>

inline void distort(float* xppOut, float* yppOut, float xp, float yp, float k1, float k2, float p1, float p2, float k3)
{
    float r2 = xp * xp + yp * yp;
    float g = std::fma(std::fma(std::fma(k3, r2, k2), r2, k1), r2, 1.0f);
    float xpp =
        xp * g + 2.0f * p1 * xp * yp + p2 * (r2 + 2.0f * xp * xp);
    float ypp =
        yp * g + p1 * (r2 + 2.0f * yp * yp) + 2.0f * p2 * xp * yp;
    *xppOut = xpp;
    *yppOut = ypp;
}
inline void invserseDistort(float* xpOut, float* ypOut, float xpp, float ypp, float k1, float k2, float p1, float p2, float k3)
{
    float xp_i = xpp;
    float yp_i = ypp;
    for (int i = 0; i < 10; ++i)
    {
        float xp2 = xp_i * xp_i;
        float yp2 = yp_i * yp_i;
        float xpyp = xp_i * yp_i;
        float r2 = xp2 + yp2;
        float g = std::fma(std::fma(std::fma(k3, r2, k2), r2, k1), r2, 1.0f);

        float nx = (xpp - (2.0f * p1 * xpyp + p2 * (r2 + 2.0f * xp2))) / g;
        float ny = (ypp - (p1 * (r2 + 2.0f * yp2) + 2.0f * p2 * xpyp)) / g;
        xp_i = nx;
        yp_i = ny;
    }
    *xpOut = xp_i;
    *ypOut = yp_i;
}

/*
    x  : intersected t. -1 is no-intersected
    yzw: un-normalized normal
*/
glm::vec4 intersect_sphere(glm::vec3 ro, glm::vec3 rd, glm::vec3 o, float r) {
    float A = glm::dot(rd, rd);
    glm::vec3 S = ro - o;
    glm::vec3 SxRD = cross(S, rd);
    float D = A * r * r - glm::dot(SxRD, SxRD);

    if (D < 0.0f) {
        return glm::vec4(-1);
    }

    float B = glm::dot(S, rd);
    float sqrt_d = sqrt(D);
    float t0 = (-B - sqrt_d) / A;
    if (0.0f < t0) {
        glm::vec3 n = (rd * t0 + S);
        return glm::vec4(t0, n);
    }

    float t1 = (-B + sqrt_d) / A;
    if (0.0f < t1) {
        glm::vec3 n = (rd * t1 + S);
        return glm::vec4(t1, n);
    }
    return glm::vec4(-1);
}
glm::vec4 combine(glm::vec4 a, glm::vec4 b) {
    if (a.x < 0.0f) {
        return b;
    }
    if (b.x < 0.0f) {
        return a;
    }
    if (a.x < b.x) {
        return a;
    }
    return b;
}

int main() {
    using namespace pr;

    SetDataDir(ExecutableDir());

    Config config;
    config.ScreenWidth = 1920;
    config.ScreenHeight = 1080;
    config.SwapInterval = 1;
    Initialize(config);

    Camera3D camera;
    camera.origin = { 4, 4, 4 };
    camera.lookat = { 0, 0, 0 };
    camera.zUp = false;

    double e = GetElapsedTime();

    // calibrated by ofxCv
    
    // camera matrix
    float fx = 3.8205359950944194e+03;
    float fy = 3.8008479819869385e+03;
    float cx = 9.5560009094769032e+02;
    float cy = 5.9411582783030872e+02;

    // distortion
    float k1 = -2.6215872723862793e-01;
    float k2 = 5.7450041695413816e+00;
    float p1 = -3.7359486345602991e-03;
    float p2 = -3.3295951797280600e-03;
    float k3 = -8.0960303437661835e+01;

    // external 

    // _MG_0066.JPG ~
    glm::mat4 externals[8];
    externals[0] = glm::mat4(0.0186952613, -0.9968802333, -0.0766832754, 0.0000000000, 0.9978028536, 0.0137267765, 0.0648151040, 0.0000000000, -0.0635602847, -0.0777265280, 0.9949465990, 0.0000000000, -8.8621168137, 7.0676698685, 86.4322814941, 1.0000000000);
    externals[1] = glm::mat4(-0.0123102767, -0.9971346259, -0.0746392459, 0.0000000000, 0.7665951252, -0.0573379956, 0.6395656466, 0.0000000000, -0.6420127153, -0.0493448563, 0.7651044130, 0.0000000000, -6.8466892242, 7.3425378799, 69.1645889282, 1.0000000000);
    externals[2] = glm::mat4(-0.1130475476, 0.8222284913, 0.5578176379, 0.0000000000, -0.9429951310, 0.0880805179, -0.3209392726, 0.0000000000, -0.3130182922, -0.5623007417, 0.7654001713, 0.0000000000, 7.5816831589, -1.5980846882, 77.4090652466, 1.0000000000);
    externals[3] = glm::mat4(-0.0065746051, -0.9942696095, -0.1066990495, 0.0000000000, 0.6963064075, 0.0720308721, -0.7141211033, 0.0000000000, 0.7177145481, -0.0789902955, 0.6918427348, 0.0000000000, -9.1275320053, 5.8227758408, 90.3689193726, 1.0000000000);
    externals[4] = glm::mat4(0.0668284744, -0.6951618195, 0.7157401443, 0.0000000000, 0.9968132973, 0.0152013646, -0.0783078894, 0.0000000000, 0.0435564294, 0.7186925411, 0.6939624548, 0.0000000000, -7.9882040024, 8.2014322281, 83.1419143677, 1.0000000000);
    externals[5] = glm::mat4(-0.0528404787, -0.9777422547, 0.2030467391, 0.0000000000, 0.8535232544, 0.0613275282, 0.5174330473, 0.0000000000, -0.5183685422, 0.2006465346, 0.8312851787, 0.0000000000, -9.4871721268, 8.5829238892, 63.0633087158, 1.0000000000);
    externals[6] = glm::mat4(-0.0563195683, -0.8838026524, -0.4644577503, 0.0000000000, 0.8327191472, 0.2150676548, -0.5102202296, 0.0000000000, 0.5508238673, -0.4154982567, 0.7238468528, 0.0000000000, -9.3425483704, 7.8671278954, 93.7747497559, 1.0000000000);
    externals[7] = glm::mat4(0.5247366428, 0.7575784922, -0.3882348537, 0.0000000000, -0.7587084770, 0.2093995064, -0.6168575883, 0.0000000000, -0.3860218525, 0.6182448864, 0.6846607924, 0.0000000000, 7.0867905617, -6.7206454277, 89.7524032593, 1.0000000000);

    // change me!
    const int number = 68;

    char filename[256];
    sprintf(filename, "checker/_MG_%04d.JPG", number);
    Image2DRGBA8 srcImage;
    srcImage.load(filename);
    
    std::vector<glm::vec3> corners;
    int w = 5;
    int h = 7;
    float squareSize = 3.0f;
    for (int i = 0; i < h; i++)
        for (int j = 0; j < w; j++)
            corners.push_back(glm::vec3(float(j * squareSize), float(i * squareSize), 0));

    const float radius = 0.3f;

    // transform points 
    auto externalParam = externals[number - 66];
    for (int i = 0; i < corners.size(); ++i)
    {
        corners[i] = externalParam * glm::vec4(corners[i], 1.0f);
    }

    float nearClip = 0.1f;
    float farClip = 10.0f;
    for (int yi = 0; yi < srcImage.height(); yi++)
    for (int xi = 0; xi < srcImage.width(); xi++)
    {
        float u = xi;
        float v = yi;

        float xpp = (u - cx) / fx;
        float ypp = (v - cy) / fy;

        float xp;
        float yp;
        invserseDistort(&xp, &yp, xpp, ypp, k1, k2, p1, p2, k3);

        glm::vec3 ro = { xp * nearClip, yp * nearClip, nearClip };
        glm::vec3 to = { xp * farClip, yp * farClip, farClip };
        glm::vec3 rd = to - ro;

        glm::vec4 isect = { -1.0f, 0, 0,0 };
        for (int i = 0; i < corners.size(); ++i)
        {
            isect = combine( isect, intersect_sphere( ro, rd, corners[i], radius) );
        }
        if (0.0f < isect.x) 
        {
            glm::vec3 n(isect.y, isect.z, isect.w);
            n = glm::normalize(n);

            glm::vec3 color = (n + glm::vec3(1.0f)) * 0.5f;
            srcImage(xi, yi) = { 255 * color.r, 255 * color.g, 255 * color.b, 255 };
        }
    }

    srcImage.saveAsPng("AR.png");

    ITexture* texture = CreateTexture();
    texture->upload(srcImage);

    while (pr::NextFrame() == false) {
        if (IsImGuiUsingMouse() == false) {
            UpdateCameraBlenderLike(&camera);
        }

        ClearBackground(0.1f, 0.1f, 0.1f, 1);

        BeginCamera(camera);

        PushGraphicState();

        DrawXYZAxis();
        DrawGrid(GridAxis::XZ, 1, 10, { 128 ,128 ,128 });

        float nearClip = 0.1f;
        float farClip = 100.0f;
        for (int yi = 0; yi < 2; yi++)
        for (int xi = 0; xi < 2; xi++)
        {
            float u = xi * srcImage.width();
            float v = yi * srcImage.height();

            float xpp = (u - cx) / fx;
            float ypp = (v - cy) / fy;

            float xp;
            float yp;
            invserseDistort(&xp, &yp, xpp, ypp, k1, k2, p1, p2, k3);

            glm::vec3 ro = { xp * nearClip, yp * nearClip, nearClip };
            glm::vec3 to = { xp * farClip, yp * farClip, farClip };
            glm::vec3 rd = to - ro;

            DrawLine(ro, to, { 255,255,255 });
        }
        for (int i = 0; i < corners.size(); ++i)
        {
            DrawSphere(corners[i], radius, { 255,0,0 });
        }

        PopGraphicState();
        EndCamera();

        BeginImGui();

        ImGui::SetNextWindowSize({ 1200, 800 }, ImGuiCond_Once);
        ImGui::Begin("Panel");
        ImGui::Text("fps = %f", GetFrameRate());
        ImGui::Image(texture, ImVec2(texture->width(), texture->height()));

        ImGui::End();

        EndImGui();
    }

    pr::CleanUp();
}
