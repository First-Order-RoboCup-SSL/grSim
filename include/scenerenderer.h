#ifndef SCENERENDERER_H
#define SCENERENDERER_H

#include "graphics.h"
#include "scenesnapshot.h"
#include <limits>

struct RenderQuality {
    bool lowSpec;
    bool showSkybox;
    bool showRobotLabels;
    static RenderQuality forLowSpec(bool enabled, bool showSkybox) {
        return {enabled, showSkybox, !enabled};
    }
};

class SceneRenderer {
public:
    explicit SceneRenderer(QGLWidget* owner) : graphics(owner) {}
    CGraphics graphics;
    void initialize(const SceneSnapshot& scene);
    void draw(const SceneSnapshot& scene, int width, int height,
              const RenderQuality& quality, int highlighted,
              bool showCursor, const std::array<double, 3>& cursor, double cursorRadius);
    static QImage robotImage(int index, const SimulationSettings& settings);
private:
    quint64 generation = std::numeric_limits<quint64>::max();
};
#endif
