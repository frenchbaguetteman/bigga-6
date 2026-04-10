/**
 * @file field_display.hpp
 * Minimap with robot trail for the INFO page.
 */
#pragma once

namespace FieldDisplay {

void init();
void clearTrail();
void draw(int x0, int y0, int size, float robotX, float robotY, float robotTheta);

}  // namespace FieldDisplay
