#include "tgTomGineThread.h"
#include <iomanip>

using namespace TomGine;

/** ThreadDrawing (OpenGL commands are only allowed in this thread) */
namespace TomGine {
void* ThreadDrawing(void* c)
{
  tgTomGineThread *tt = (tgTomGineThread*) c;

  TomGine::tgEngine* render = new TomGine::tgEngine(tt->m_width, tt->m_height, tt->m_depth_max,
      tt->m_depth_min, tt->m_windowname.c_str(), tt->m_bfc, true);

  pthread_mutex_lock(&tt->dataMutex);
  tt->m_engine = render;
  bool stop = tt->m_stopTomGineThread;
  pthread_mutex_unlock(&tt->dataMutex);

  std::list<Event> eventlist;
  float fTime;
  unsigned fps;

  while (!stop) {
    if (!tt->m_FPS)
      sem_wait(&tt->renderSem);

    pthread_mutex_lock(&tt->dataMutex);
    if (tt->m_clear) {
      tt->GL_Clear();
      sem_post(&tt->clearFinishedSem);
    }
    pthread_mutex_unlock(&tt->dataMutex);

    pthread_mutex_lock(&tt->eventMutex);
    eventlist.clear();
    while (!tt->m_eventlist.empty()) {
      eventlist.push_back(tt->m_eventlist.front());
      tt->m_eventlist.pop_front();
      sem_trywait(&tt->renderSem);
    }
    tt->m_eventlist.clear();
    pthread_mutex_unlock(&tt->eventMutex);

    pthread_mutex_lock(&tt->dataMutex);
    while (!eventlist.empty()) {
      render->InputControl(eventlist.front());
      eventlist.pop_front();
    }
    tt->GL_Update(render);
    tt->GL_Draw(render);

    stop = tt->m_stopTomGineThread;
    pthread_mutex_unlock(&tt->dataMutex);

    if (tt->m_waitForZBuffer) {
      pthread_mutex_lock(&tt->dataMutex);
      tt->GL_ZBuffer(tt->m_zBuffer, render);
      tt->m_waitForZBuffer = false;
      pthread_mutex_unlock(&tt->dataMutex);
    }
    if (tt->m_snapshot) {
      pthread_mutex_lock(&tt->dataMutex);
      tt->GL_Snapshot(tt->m_snapshotFile);
      tt->m_snapshot = false;
      pthread_mutex_unlock(&tt->dataMutex);
    }
    if (tt->m_record) {
      pthread_mutex_lock(&tt->dataMutex);
      tt->GL_Record(render);
      pthread_mutex_unlock(&tt->dataMutex);
    }

    render->Update(fTime);

    if (tt->m_FPS) {
      fps = unsigned(round(1.0 / fTime));
      pthread_mutex_lock(&tt->dataMutex);
      if (fps > tt->m_maxFPS)
        tt->m_fTime += 0.001;
      else if (fps < tt->m_maxFPS)
        tt->m_fTime -= 0.001;
      tt->m_fTime < 0.0 ? fTime = -tt->m_fTime : fTime = tt->m_fTime;
      pthread_mutex_unlock(&tt->dataMutex);
      usleep(unsigned(fTime * 1e6));
    }

    sem_post(&tt->renderFinishedSem);

  }

  pthread_mutex_lock(&tt->dataMutex);
  tt->GL_Clear();
  tt->m_engine = NULL;
  pthread_mutex_unlock(&tt->dataMutex);

  render->UnWaitForEvent();
  delete render;

  pthread_mutex_lock(&tt->dataMutex);
  tt->m_renderingStopped = true;
  pthread_mutex_unlock(&tt->dataMutex);

  sem_post(&tt->renderFinishedSem);

  pthread_exit(NULL);
}

void* ThreadEventHandling(void* c)
{
  tgTomGineThread *tt = (tgTomGineThread*) c;

  pthread_mutex_lock(&tt->dataMutex);
  bool stop = tt->m_stopTomGineThread;
  pthread_mutex_unlock(&tt->dataMutex);

  bool engineExists = false;

  Event event;
  while (!stop) {
    if (engineExists) {

      tt->m_engine->WaitForEvent(event);

      pthread_mutex_lock(&tt->dataMutex);
      if (tt->KeyHandler(event))
        tt->m_stopTomGineThread = true;
      pthread_mutex_unlock(&tt->dataMutex);

      pthread_mutex_lock(&tt->eventMutex);
      tt->m_eventlist.push_back(event);

      if (tt->m_waitingForEvents)
        if (event.type == TMGL_Press || event.type == TMGL_Release) {
          tt->m_waitingeventlist.push_back(event);
        }

      if (tt->m_listenForEvents) {
        //        if (event.type == TMGL_Press || event.type == TMGL_Release) {
        tt->m_listenEventList.push_front(event);
        if (tt->m_listenEventList.size() > tt->m_listenEventListSize) // avoid out-of-memory
          tt->m_listenEventList.resize(tt->m_listenEventListSize);
        //        }
      }
      sem_post(&tt->renderSem);
      pthread_mutex_unlock(&tt->eventMutex);

    } else {
      usleep(10000); // wait for render engine to start up
    }
    pthread_mutex_lock(&tt->dataMutex);
    stop = tt->m_stopTomGineThread;
    engineExists = tt->m_engine;
    pthread_mutex_unlock(&tt->dataMutex);
  }

  pthread_mutex_lock(&tt->dataMutex);
  tt->m_eventsStopped = true;
  pthread_mutex_unlock(&tt->dataMutex);

  pthread_exit(NULL);
}
}

/********************** tgTomGineThread ************************/
void tgTomGineThread::init()
{
  m_stopTomGineThread = false;
  m_renderingStopped = false;
  m_eventsStopped = false;
  m_clear = false;
  m_clearColor = TomGine::vec3(0.0f, 0.0f, 0.0f);
  m_maxFPS = 0;
  m_FPS = false;
  m_fTime = 0.0;
  m_snapshot = false;
  m_record = false;
  m_recorderFrame = 0;
  m_numScreenShot = 0;

  df.image = true;
  df.pointcloud = true;
  df.lines = true;
  df.points = true;
  df.labels = true;
  df.models = true;
  df.nurbs = true;

  m_loadImage = false;
  m_showCoordinateFrame = false;
  m_camChanged = false;
  m_tgCamChanged = false;
  m_inputSpeedChanged = false;
  m_rotCenterChanged = false;
  m_waitingForEvents = false;
  m_listenForEvents = false;
  m_waitForZBuffer = false;

  m_listenEventListSize = 100;

  m_engine = NULL;

  sem_init(&renderSem, 0, 0);
  sem_init(&renderFinishedSem, 0, 0);
  sem_init(&clearFinishedSem, 0, 0);
  sem_init(&snapshotSem, 0, 0);
  pthread_mutex_init(&eventMutex, NULL);
  pthread_mutex_init(&dataMutex, NULL);
  pthread_create(&thread_gl, NULL, ThreadDrawing, this);
  pthread_create(&thread_event, NULL, ThreadEventHandling, this);
}

tgTomGineThread::tgTomGineThread(int w, int h, std::string windowname, bool bfc, float depth_min,
    float depth_max) :
  m_width(w), m_height(h), m_depth_min(depth_min), m_depth_max(depth_max), m_bfc(bfc),
      m_windowname(windowname)
{
  init();
}

tgTomGineThread::~tgTomGineThread()
{
  m_stopTomGineThread = true;
  sem_post(&renderSem);
  sem_post(&renderFinishedSem);

  pthread_join(thread_gl, NULL);
  pthread_join(thread_event, NULL);

  sem_destroy(&renderSem);
  sem_destroy(&renderFinishedSem);
  sem_destroy(&clearFinishedSem);
  sem_destroy(&snapshotSem);
  pthread_mutex_destroy(&dataMutex);
  pthread_mutex_destroy(&eventMutex);
}

void tgTomGineThread::GL_Clear()
{
  for (unsigned i = 0; i < this->m_pointclouds.size(); i++)
    delete this->m_pointclouds[i];
  m_pointclouds.clear();
  for (unsigned i = 0; i < this->m_nurbsSurface.size(); i++)
    delete this->m_nurbsSurface[i];
  m_nurbsSurface.clear();
  m_nurbsSurfaceData.clear();
  for (unsigned i = 0; i < this->m_textures_gl.size(); i++)
    delete this->m_textures_gl[i];
  m_textures_gl.clear();
  m_textures.clear();

  m_points2D.clear();
  m_pointSize2D.clear();

  m_points3D.clear();
  m_pointSize3D.clear();

  m_lines2D.clear();
  m_lineCols2D.clear();
  m_lineWidth2D.clear();

  m_lines3D.clear();
  m_lineCols3D.clear();
  m_lineWidth3D.clear();

  m_labels2D.clear();
  m_labels3D.clear();

  m_modelpointers.clear();
  m_models2D.clear();
  m_models3D.clear();
}

void tgTomGineThread::PrintUsage()
{
  printf("\n\n --- TomGine Render Engine ---\n\n");
  printf("  [Escape,q] Quit\n");
  printf("  [f] Show / Hide coordinate frame\n");
  printf("  [d] Show / Hide points (dots)\n");
  printf("  [i] Show / Hide background image\n");
  printf("  [p] Show / Hide point-clouds\n");
  printf("  [t] Show / Hide labels (text)\n");
  printf("  [l] Show / Hide lines\n");
  printf("  [m] Show / Hide models\n");
  printf("  [o] Print camera info\n");
  printf("  [r] Print current center of rotation\n");
  printf("  [F11] Take screenshot\n");
  printf("\n\n");
}

bool tgTomGineThread::KeyHandler(Event &event)
{
  if (event.type == TMGL_Quit)
    return true;
  if (event.type == TMGL_Press) {
    if (event.input == TMGL_Escape || event.input == TMGL_q)
      return true;
    //  [clear] internal events should not manipulate data (external processes might need it)
    else if (event.input == TMGL_f)
      m_showCoordinateFrame = !m_showCoordinateFrame;
    else if (event.input == TMGL_d)
      df.points = !df.points;
    else if (event.input == TMGL_i)
      df.image = !df.image;
    else if (event.input == TMGL_p)
      df.pointcloud = !df.pointcloud;
    else if (event.input == TMGL_t)
      df.labels = !df.labels;
    else if (event.input == TMGL_l)
      df.lines = !df.lines;
    else if (event.input == TMGL_m)
      df.models = !df.models;
    //    else if (event.input == TMGL_n)
    //      df.nurbs = !df.nurbs;
    else if (event.input == TMGL_o)
      m_engine->GetCamera().Print();
    else if (event.input == TMGL_r)
      printf("COR: %f %f %f\n", m_rotCenter.x, m_rotCenter.y, m_rotCenter.z);
    else if (event.input == TMGL_F11) {
      std::ostringstream os;
      os << "tgImg_" << std::setfill('0') << std::setw(4) << this->m_numScreenShot++ << ".png";
      m_snapshotFile = os.str();
      m_snapshot = true;
    }
  }
  return false;
}

void tgTomGineThread::GL_DrawPointCloud()
{

  glEnable(GL_POINT_SMOOTH);
  glPointSize(1.0);

  for (unsigned i = 0; i < this->m_pointclouds.size(); i++){
    this->m_pointclouds[i]->DrawColorPoints();
    this->m_pointclouds[i]->DrawLines();
  }

#ifdef DEBUG
  tgCheckError("[tgTomGineThread::DrawPointCloud]");
#endif
}

void tgTomGineThread::GL_DrawPoints3D()
{
  glDisable(GL_LIGHTING);
  glEnable(GL_POINT_SMOOTH);

  for (unsigned i = 0; i < m_points3D.size(); i++) {
    glPointSize(m_pointSize3D[i]);
    glBegin(GL_POINTS);
    glColor3ub(m_points3D[i].color[0], m_points3D[i].color[1], m_points3D[i].color[2]);
    glVertex3f(m_points3D[i].pos.x, m_points3D[i].pos.y, m_points3D[i].pos.z);
    glEnd();
  }
  glPointSize(1.0f);

#ifdef DEBUG
  tgCheckError("[tgTomGineThread::DrawPoints3D]");
#endif
}

void tgTomGineThread::GL_DrawPoints2D()
{
  glDisable(GL_LIGHTING);
  glEnable(GL_POINT_SMOOTH);

  for (unsigned i = 0; i < m_points2D.size(); i++) {
    glPointSize(m_pointSize2D[i]);
    glBegin(GL_POINTS);
    glColor3ub(m_points2D[i].color[0], m_points2D[i].color[1], m_points2D[i].color[2]);
    glVertex3f(m_points2D[i].pos.x, m_points2D[i].pos.y, m_points2D[i].pos.z);
    glEnd();
  }
  glPointSize(1.0f);

#ifdef DEBUG
  tgCheckError("[tgTomGineThread::DrawPoints2D]");
#endif
}

void tgTomGineThread::GL_DrawLines2D()
{
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);

  for (unsigned i = 0; i < m_lines2D.size(); i++) {
    glLineWidth(m_lineWidth2D[i]);
    glBegin(GL_LINES);
    glColor3f(m_lineCols2D[i].x, m_lineCols2D[i].y, m_lineCols2D[i].z);
    glVertex3f(m_lines2D[i].start.x, m_lines2D[i].start.y, m_lines2D[i].start.z);
    glVertex3f(m_lines2D[i].end.x, m_lines2D[i].end.y, m_lines2D[i].end.z);
    glEnd();
  }
  glLineWidth(1.0f);
#ifdef DEBUG
  tgCheckError("[tgTomGineThread::DrawLines2D]");
#endif
}

void tgTomGineThread::GL_DrawLines3D()
{
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);

  for (unsigned i = 0; i < m_lines3D.size(); i++) {
    glLineWidth(m_lineWidth3D[i]);
    glBegin(GL_LINES);
    glColor3f(m_lineCols3D[i].x, m_lineCols3D[i].y, m_lineCols3D[i].z);
    glVertex3f(m_lines3D[i].start.x, m_lines3D[i].start.y, m_lines3D[i].start.z);
    glVertex3f(m_lines3D[i].end.x, m_lines3D[i].end.y, m_lines3D[i].end.z);
    glEnd();
  }
  glLineWidth(1.0f);
#ifdef DEBUG
  tgCheckError("[tgTomGineThread::DrawLines3D]");
#endif
}

void tgTomGineThread::GL_DrawLabels(TomGine::tgEngine *render)
{
  render->Activate3D();
  mat4 modelview, projection;
  vec4 viewport;
  glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
  glGetFloatv(GL_PROJECTION_MATRIX, projection);
  glGetFloatv(GL_VIEWPORT, viewport);
  mat4 modelviewprojection = projection * modelview;

  render->Activate2D();
  for (unsigned i = 0; i < this->m_labels3D.size(); i++) {
    tgLabel3D l = m_labels3D[i];
    vec4 texCoords = modelviewprojection * vec4(l.pos.x, l.pos.y, l.pos.z, 1.0);
    float x = (texCoords.x / texCoords.w + 1.0f) * 0.5f;
    float y = (texCoords.y / texCoords.w + 1.0f) * 0.5f;

    g_font->Print(l.text.c_str(), l.size, int(viewport.z * x), int(viewport.w * y), l.rgba.x, l.rgba.y, l.rgba.z, l.rgba.w);
  }

  for (unsigned i = 0; i < this->m_labels2D.size(); i++) {
    tgLabel2D l = m_labels2D[i];
    g_font->Print(l);
  }
}

void tgTomGineThread::GL_DrawModels2D()
{
  for (unsigned i = 0; i < this->m_models2D.size(); i++) {
    m_models2D[i].Draw();
  }
#ifdef DEBUG
  tgCheckError("[tgTomGineThread::GL_DrawModels2D]");
#endif
}

void tgTomGineThread::GL_DrawModels3D()
{
  for (unsigned i = 0; i < this->m_modelpointers.size(); i++) {
    m_modelpointers[i]->Draw();
  }
  for (unsigned i = 0; i < this->m_models3D.size(); i++) {
    m_models3D[i].Draw();
  }
#ifdef DEBUG
  tgCheckError("[tgTomGineThread::GL_DrawModels3D]");
#endif
}

//void tgTomGineThread::GL_SyncNurbsData()
//{
//  // add new nurbs
//  while (this->m_nurbsSurface.size() < this->m_nurbsSurfaceData.size()) {
//    unsigned id = this->m_nurbsSurface.size();
//    TomGine::tgNurbsSurface *surf = new TomGine::tgNurbsSurface(this->m_nurbsSurfaceData[id],
//        m_engine->m_NurbsSurfaceShader);
//    this->m_nurbsSurface.push_back(surf);
//    this->m_nurbsSurfaceData[id].sync = true;
//  }
//#ifdef DEBUG
//  tgCheckError("[tgTomGineThread::SyncNurbsData] add new nurbs");
//#endif
//  // update out of sync nurbs
//  for (unsigned i = 0; i < this->m_nurbsSurfaceData.size(); i++) {
//    if (!this->m_nurbsSurfaceData[i].sync) {
//      this->m_nurbsSurface[i]->Set(this->m_nurbsSurfaceData[i]);
//      this->m_nurbsSurfaceData[i].sync = true;
//    }
//  }
//
//}
//
//void tgTomGineThread::GL_DrawNurbs()
//{
//  for (unsigned i = 0; i < this->m_nurbsSurface.size(); i++) {
//    glLineWidth(1.0f);
//    this->m_nurbsSurface[i]->DrawFaces();
//    glPointSize(5.0);
//    glColor3f(0.6, 0.0, 0.6);
//    this->m_nurbsSurface[i]->DrawCPs();
//  }
//#ifdef DEBUG
//  tgCheckError("[tgTomGineThread::DrawNurbs]");
//#endif
//}

void tgTomGineThread::GL_Record(TomGine::tgEngine *render)
{
  char img_number[16];
  sprintf(img_number, "img-%06d.jpg", m_recorderFrame++);
  std::string filename = m_recorderPath;
  filename.append(img_number);
  GL_Snapshot(filename);

  render->Activate2D();
  glPointSize(20);
  glBegin(GL_POINTS);
  glColor3ub(255, 0, 0);
  glVertex3f(m_width - 15, m_height - 15, 0.0);
  glEnd();

  render->DrawFPS();
}

void tgTomGineThread::GL_Snapshot(const std::string &filename)
{
  TomGine::tgTexture2D tex;
  tex.CopyTexImage2D(m_width, m_height, GL_RGB);
  tex.Save(filename.c_str());
  printf("[tgTomGineThread::GL_Snapshot] Saved image to '%s'\n", filename.c_str());
  sem_post(&snapshotSem);
}

void tgTomGineThread::GL_ZBuffer(cv::Mat1f &z, TomGine::tgEngine *render)
{
  z = cv::Mat1f(m_height, m_width);
  if (!z.isContinuous())
    printf("[tgTomGineThread::GL_ZBuffer] Warning, cv::Mat1f is not memory aligned.\n");

  float far = render->GetCamera().GetZFar();
  float near = render->GetCamera().GetZNear();
  float c = (far + near) / (far - near);
  float d = (2 * far * near) / (far - near);

  glReadPixels(0, 0, m_width, m_height, GL_DEPTH_COMPONENT, GL_FLOAT, &z(0, 0));

  for (int i = 0; i < z.rows; i++)
    for (int j = 0; j < z.cols; j++) {
      //      z(i, j) = d / (c - z(i, j));
      float &val = z(i, j);
      val = 2.0f * val - 1.0f;
      val = d / (c - val);
    }
}

void tgTomGineThread::GL_Update(TomGine::tgEngine *render)
{
  glClearColor(m_clearColor.x, m_clearColor.y, m_clearColor.z, 1.);

  // set camera pose...
  if (m_camChanged) {
    unsigned w = unsigned(m_width);
    unsigned h = unsigned(m_height);
    render->SetCamera(m_intrinsic, w, h, m_extR, m_extT);
    m_camChanged = false;
#ifdef DEBUG
    tgCheckError("[tgTomGineThread::GL_Update] SetCamera (cv):");
#endif
  }
  if (m_tgCamChanged) {
    render->SetCamera(m_tgCamera);
    m_tgCamChanged = false;
#ifdef DEBUG
    tgCheckError("[tgTomGineThread::GL_Update] SetCamera (tg):");
#endif
  }
  if (m_inputSpeedChanged) {
    render->SetSpeeds(m_inputSpeeds.x, m_inputSpeeds.y, m_inputSpeeds.z);
    m_inputSpeedChanged = false;
  }
  // set center of rotation
  if (m_rotCenterChanged) {
    render->SetCenterOfRotation(m_rotCenter.x, m_rotCenter.y, m_rotCenter.z);
#ifdef DEBUG
    tgCheckError("[tgTomGineThread::GL_Update]");
#endif
  }
  // synchronize NURBS data
  //  if (df.nurbs)
  //    GL_SyncNurbsData();

}

void tgTomGineThread::GL_Draw(TomGine::tgEngine *render)
{
  if (m_loadImage && !m_img.empty()) {
    if(m_img.channels()==1)
          render->LoadBackgroundImage(m_img.data, m_img.cols, m_img.rows, GL_LUMINANCE, true);
    if(m_img.channels()==3)
          render->LoadBackgroundImage(m_img.data, m_img.cols, m_img.rows, GL_BGR, true);
    m_loadImage = false;
  }

  render->Activate2D();
  if (df.image)
    render->DrawBackgroundImage();

  if (df.points)
    GL_DrawPoints2D();

  if (df.lines)
    GL_DrawLines2D();

  if (df.models) {
    GL_DrawModels2D();
  }

  render->Activate3D();

  if (df.models) {
    GL_DrawModels3D();
    //    GL_DrawTexModels();
  }

  //  if (df.nurbs)
  //    GL_DrawNurbs();

  if (df.pointcloud)
    GL_DrawPointCloud();

  if (df.points)
    GL_DrawPoints3D();

  if (df.lines)
    GL_DrawLines3D();

  if (m_showCoordinateFrame)
    render->DrawCoordinates(1.0, 2.0);

  if (df.labels)
    GL_DrawLabels(render);

}

/***************************** PUBLIC *****************************/

Event tgTomGineThread::WaitForEvent(Type type, Input input)
{
  Event event;
  bool valid = false;
  while (!valid && !this->m_renderingStopped) {
    pthread_mutex_lock(&eventMutex);
    m_waitingForEvents = true;
    pthread_mutex_unlock(&eventMutex);

    sem_wait(&renderFinishedSem);
    pthread_mutex_lock(&eventMutex);
    m_waitingForEvents = false;
    while (!m_waitingeventlist.empty()) {
      event = m_waitingeventlist.front();
      m_waitingeventlist.pop_front();

      if (event.type == type) {
        if (event.input == input) {
          valid = true;
        }
      }

    }
    pthread_mutex_unlock(&eventMutex);
  }

  if (!valid)
    event.type = TMGL_Quit;

  return event;
}

void tgTomGineThread::StartEventListener(unsigned max_events)
{
  pthread_mutex_lock(&eventMutex);
  this->m_listenForEvents = true;
  this->m_listenEventListSize = max_events;
  pthread_mutex_unlock(&eventMutex);
}

void tgTomGineThread::StopEventListener()
{
  pthread_mutex_lock(&eventMutex);
  this->m_listenForEvents = false;
  pthread_mutex_unlock(&eventMutex);
}

void tgTomGineThread::GetEventQueue(std::list<Event> &events)
{
  pthread_mutex_lock(&eventMutex);
  events = this->m_listenEventList;
  this->m_listenEventList.clear();
  pthread_mutex_unlock(&eventMutex);
}

bool ascending(double i, double j)
{
  return (i < j);
}

bool tgTomGineThread::SelectModel(int x, int y, int &id)
{
  if (this->m_renderingStopped)
    return false;

  bool intersection(false);
  tgRay ray;
  GetCamera().GetViewRay(x, y, ray.start, ray.dir);

  pthread_mutex_lock(&dataMutex);

  // Get shortest distance to each model
  std::vector<double> dist(m_models3D.size(), DBL_MAX);
  for (unsigned i = 0; i < m_models3D.size(); i++) {
    std::vector<vec3> pl, nl;
    std::vector<double> zl;
    bool tmp_intersect = tgCollission::IntersectRayModel(pl, nl, zl, ray, m_models3D[i]);
    intersection = (intersection || tmp_intersect);
    if (tmp_intersect) {
      double z_min(DBL_MAX);
      for (unsigned j = 0; j < zl.size(); j++)
        if (zl[j] < z_min)
          z_min = zl[j];

      dist[i] = z_min;
    }
  }

  if (intersection) {
    double z_min(DBL_MAX);
    for (unsigned i = 0; i < dist.size(); i++) {
      if (dist[i] < z_min) {
        z_min = dist[i];
        id = i;
      }
    }
  }
  pthread_mutex_unlock(&dataMutex);

  return intersection;
}

TomGine::tgCamera tgTomGineThread::GetCamera()
{
  sem_post(&renderSem);
  sem_wait(&renderFinishedSem);
  return this->m_engine->GetCamera();
}

void tgTomGineThread::SetClearColor(float r, float g, float b)
{
  pthread_mutex_lock(&dataMutex);
  m_clearColor = TomGine::vec3(r, g, b);
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetClearColor(float gray)
{
  pthread_mutex_lock(&dataMutex);
  m_clearColor = TomGine::vec3(gray, gray, gray);
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetCamera(cv::Mat &_intrinsic)
{
  pthread_mutex_lock(&dataMutex);
  _intrinsic.copyTo(m_intrinsic);
  m_camChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetCamera(cv::Mat &R, cv::Mat &t)
{
  pthread_mutex_lock(&dataMutex);
  R.copyTo(m_extR);
  t.copyTo(m_extT);
  m_camChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetCamera(const TomGine::tgCamera &cam)
{
  pthread_mutex_lock(&dataMutex);
  m_tgCamera = cam;
  m_tgCamChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetCameraDefault()
{
  pthread_mutex_lock(&dataMutex);
  m_extR = cv::Mat::eye(3, 3, CV_32F);
  m_extT = cv::Mat::zeros(3, 1, CV_32F);
  m_intrinsic = cv::Mat::zeros(3, 3, CV_64F);
  m_intrinsic.at<double> (0, 0) = m_intrinsic.at<double> (1, 1) = 525;
  m_intrinsic.at<double> (0, 2) = 320;
  m_intrinsic.at<double> (1, 2) = 240;
  m_intrinsic.at<double> (2, 2) = 1;
  m_camChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetRotationCenter(const cv::Vec3d &_rotCenter)
{
  pthread_mutex_lock(&dataMutex);
  m_rotCenter = vec3((float) _rotCenter[0], (float) _rotCenter[1], (float) _rotCenter[2]);
  m_rotCenterChanged = true;
  pthread_mutex_unlock(&dataMutex);
}
void tgTomGineThread::SetRotationCenter(float x, float y, float z)
{
  pthread_mutex_lock(&dataMutex);
  m_rotCenter = vec3(x, y, z);
  m_rotCenterChanged = true;
  pthread_mutex_unlock(&dataMutex);
}
void tgTomGineThread::SetRotationCenter(const TomGine::vec3 &cor)
{
  pthread_mutex_lock(&dataMutex);
  m_rotCenter = cor;
  m_rotCenterChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetCoordinateFrame()
{
  pthread_mutex_lock(&dataMutex);
  m_showCoordinateFrame = !m_showCoordinateFrame;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetImage(const cv::Mat &_img, bool show, bool background)
{
  if (_img.channels() != 3 && _img.channels() != 1) {
    printf("[tgTomGineThread::SetImage] Warning, image is no rgb image\n");
    return;
  }

  pthread_mutex_lock(&dataMutex);
  _img.copyTo(m_img);
  m_loadImage = true;
  df.image = show;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetInputSpeeds(float rotation, float translation, float zoom)
{
  pthread_mutex_lock(&dataMutex);
  m_inputSpeeds = vec3(rotation, translation, zoom);
  m_inputSpeedChanged = true;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetFrameRate(unsigned max_fps)
{
  pthread_mutex_lock(&dataMutex);
  m_maxFPS = max_fps;
  m_FPS = !(m_maxFPS == 0);
  pthread_mutex_unlock(&dataMutex);
}

int tgTomGineThread::AddPoint2D(float x, float y, uchar r, uchar g, uchar b, float size)
{
  if (size <= 0.0f) {
    printf("tgTomGineThread::AddPoint2D] Warning invalid argument: size <= 0.0f\n");
    size = 1.0f;
  }
  pthread_mutex_lock(&dataMutex);
  tgColorPoint pt;
  pt.pos = vec3(x, y, 0.0f);
  pt.color[0] = r;
  pt.color[1] = g;
  pt.color[2] = b;
  m_points2D.push_back(pt);
  m_pointSize2D.push_back(size);
  int id = (this->m_points2D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddPoint3D(float x, float y, float z, uchar r, uchar g, uchar b, float size)
{
  if (size <= 0.0f) {
    printf("tgTomGineThread::AddPoint3D] Warning invalid argument: size <= 0.0f\n");
    size = 1.0f;
  }
  pthread_mutex_lock(&dataMutex);
  tgColorPoint pt;
  pt.pos = vec3(x, y, z);
  pt.color[0] = r;
  pt.color[1] = g;
  pt.color[2] = b;
  m_points3D.push_back(pt);
  m_pointSize3D.push_back(size);
  int id = (this->m_points3D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddLine2D(float x1, float y1, float x2, float y2, uchar r, uchar g, uchar b,
    float width)
{
  if (width <= 0.0f) {
    printf("tgTomGineThread::AddLine2D] Warning invalid argument: width <= 0.0f\n");
    width = 1.0f;
  }
  pthread_mutex_lock(&dataMutex);
  m_lines2D.push_back(TomGine::tgLine(vec3(x1, y1, 0.0), vec3(x2, y2, 0.0)));
  m_lineCols2D.push_back(vec3(float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f));
  m_lineWidth2D.push_back(width);
  int id = (this->m_lines2D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddLine3D(float x1, float y1, float z1, float x2, float y2, float z2, uchar r,
    uchar g, uchar b, float width)
{
  if (width <= 0.0f) {
    printf("tgTomGineThread::AddLine3D] Warning invalid argument: width <= 0.0f\n");
    width = 1.0f;
  }
  pthread_mutex_lock(&dataMutex);
  m_lines3D.push_back(TomGine::tgLine(vec3(x1, y1, z1), vec3(x2, y2, z2)));
  m_lineCols3D.push_back(vec3(float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f));
  m_lineWidth3D.push_back(width);
  int id = (this->m_lines3D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddLabel2D(std::string text, int size, int x, int y, float r, float g, float b)
{
  pthread_mutex_lock(&dataMutex);
  m_labels2D.push_back(TomGine::tgLabel2D(text, size, x, y, r, g, b));
  int id = (this->m_labels2D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddLabel3D(std::string text, int size, vec3 pos, float r, float g, float b)
{
  pthread_mutex_lock(&dataMutex);
  TomGine::tgLabel3D label;
  label.text = text;
  label.size = size;
  label.pos = pos;
  label.rgba = vec4(r, g, b, 1.0f);
  m_labels3D.push_back(label);
  int id = (this->m_labels3D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddLabel3D(std::string text, int size, double x, double y, double z)
{
  pthread_mutex_lock(&dataMutex);
  TomGine::tgLabel3D label;
  label.text = text;
  label.size = size;
  label.pos.x = x;
  label.pos.y = y;
  label.pos.z = z;
  m_labels3D.push_back(label);
  int id = (this->m_labels3D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddModel2D(const TomGine::tgTextureModel &model)
{
  pthread_mutex_lock(&dataMutex);
  this->m_models2D.push_back(model);
  int id = (this->m_models2D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddModel3D(const TomGine::tgTextureModel &model)
{
  pthread_mutex_lock(&dataMutex);
  this->m_models3D.push_back(model);
  int id = (this->m_models3D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddModel(const TomGine::tgTextureModel &model)
{
  pthread_mutex_lock(&dataMutex);
  this->m_models3D.push_back(model);
  int id = (this->m_models3D.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddModel(TomGine::tgModel *model)
{
  pthread_mutex_lock(&dataMutex);
  this->m_modelpointers.push_back(model);
  int id = (this->m_modelpointers.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddNurbsSurface(const TomGine::tgNurbsSurfacePatch &nurbsData)
{
  pthread_mutex_lock(&dataMutex);
  this->m_nurbsSurfaceData.push_back(nurbsData);
  int id = (this->m_nurbsSurfaceData.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddPointCloud(const TomGine::tgModel &pcl)
{
  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_colorpoints = pcl.m_colorpoints;

  pthread_mutex_lock(&dataMutex);
  this->m_pointclouds.push_back(tg_cloud);
  int id = (this->m_pointclouds.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddPointCloud(cv::Mat_<cv::Vec4f> cloud)
{
  tgRGBValue color;
  tgModel* tg_cloud = new tgModel;

  for (int i = 0; i < cloud.rows; i++) {
    for (int j = 0; j < cloud.cols; j++) {
      TomGine::tgColorPoint cpt;
      cv::Vec4f &pt = cloud(i, j);
      color.float_value = pt[3];
      cpt.color[0] = color.Red;
      cpt.color[1] = color.Green;
      cpt.color[2] = color.Blue;
      cpt.pos = vec3(pt[0], pt[1], pt[2]);
      tg_cloud->m_colorpoints.push_back(cpt);
    }
  }
  pthread_mutex_lock(&dataMutex);
  this->m_pointclouds.push_back(tg_cloud);
  int id = (this->m_pointclouds.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

int tgTomGineThread::AddPointCloud(const std::vector<cv::Vec4f> &cloud)
{
  tgRGBValue color;
  tgModel* tg_cloud = new tgModel;

  for (unsigned i = 0; i < cloud.size(); i++) {
    TomGine::tgColorPoint cpt;
    const cv::Vec4f &pt = cloud[i];
    color.float_value = pt[3];
    cpt.color[0] = color.Red;
    cpt.color[1] = color.Green;
    cpt.color[2] = color.Blue;
    cpt.pos = vec3(pt[0], pt[1], pt[2]);
    tg_cloud->m_colorpoints.push_back(cpt);
  }
  pthread_mutex_lock(&dataMutex);
  this->m_pointclouds.push_back(tg_cloud);
  int id = (this->m_pointclouds.size() - 1);
  pthread_mutex_unlock(&dataMutex);
  return id;
}

void tgTomGineThread::SetPoint2D(int id, float x, float y)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_points2D.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetPoint2D] Warning index out of bounds: %d.\n", id);
    return;
  }
  TomGine::tgColorPoint &pt = this->m_points2D[id];
  pt.pos.x = x;
  pt.pos.y = y;
  pthread_mutex_unlock(&dataMutex);
}
void tgTomGineThread::SetPoint3D(int id, float x, float y, float z)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_points3D.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetPoint3D] Warning index out of bounds: %d.\n", id);
    return;
  }
  this->m_points3D[id].pos.x = x;
  this->m_points3D[id].pos.y = y;
  this->m_points3D[id].pos.z = z;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetLabel2D(int id, std::string text, int size, int x, int y, float r,
    float g, float b)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_labels2D.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetLabel2D] Warning index out of bounds: %d.\n", id);
    return;
  }
  this->m_labels2D[id] = TomGine::tgLabel2D(text, size, x, y, r, g, b);
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetModel2D(int id, const TomGine::tgTextureModel &model)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_models2D.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetModel2D] Warning index out of bounds: %d.\n", id);
    return;
  }
  this->m_models2D[id] = model;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetModel(int id, const TomGine::tgTextureModel &model)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_models3D.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetModel] Warning index out of bounds: %d.\n", id);
    return;
  }
  this->m_models3D[id] = model;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetModelPose(int id, const TomGine::tgPose &pose)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_models3D.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetModel] Warning index out of bounds: %d.\n", id);
    return;
  }
  this->m_models3D[id].m_pose = pose;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetModel(int id, TomGine::tgModel *model)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_modelpointers.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetModel] Warning index out of bounds: %d.\n", id);
    return;
  }
  this->m_modelpointers[id] = model;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetNurbsSurface(int id, const TomGine::tgNurbsSurfacePatch &nurbsData)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_nurbsSurfaceData.size()) {
    pthread_mutex_unlock(&dataMutex);
    printf("[tgTomGineThread::SetNurbsSurface] Warning index out of bounds: %d.\n", id);
    return;
  }
  this->m_nurbsSurfaceData[id] = nurbsData;
  this->m_nurbsSurfaceData[id].sync = false;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetPointCloud(int id, const TomGine::tgModel &pcl)
{
  if (this->m_renderingStopped)
    return;

  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_colorpoints = pcl.m_colorpoints;

  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_pointclouds.size()) {
    pthread_mutex_unlock(&dataMutex);
    if (this->m_renderingStopped)
      return;
    printf("[tgTomGineThread::SetPointCloud] Warning index out of bounds: %d.\n", id);
    return;
  }
  delete this->m_pointclouds[id];
  this->m_pointclouds[id] = tg_cloud;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetPointCloud(int id, cv::Mat_<cv::Vec4f> cloud)
{
  if (this->m_renderingStopped)
    return;

  tgRGBValue color;
  tgModel* tg_cloud = new tgModel;

  for (int i = 0; i < cloud.rows; i++) {
    for (int j = 0; j < cloud.cols; j++) {
      TomGine::tgColorPoint cpt;
      cv::Vec4f &pt = cloud(i, j);
      color.float_value = pt[3];
      cpt.color[0] = color.Red;
      cpt.color[1] = color.Green;
      cpt.color[2] = color.Blue;
      cpt.pos = vec3(pt[0], pt[1], pt[2]);
      tg_cloud->m_colorpoints.push_back(cpt);
    }
  }

  pthread_mutex_lock(&dataMutex);
  if (id < 0 || id >= (int) this->m_pointclouds.size()) {
    if (this->m_renderingStopped) {
      pthread_mutex_unlock(&dataMutex);
      return;
    } else {
      pthread_mutex_unlock(&dataMutex);
      printf("[tgTomGineThread::SetPointCloud] Warning index out of bounds: %d.\n", id);
      return;
    }
  }
  delete this->m_pointclouds[id];
  this->m_pointclouds[id] = tg_cloud;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::Update()
{
  if (this->m_renderingStopped)
    return;
  Event event;
  event.type = TMGL_None;
  pthread_mutex_lock(&eventMutex);
  m_eventlist.push_back(event);
  sem_post(&renderSem);
  pthread_mutex_unlock(&eventMutex);

  sem_wait(&renderFinishedSem);
}

bool tgTomGineThread::Stopped()
{
  pthread_mutex_lock(&dataMutex);
  bool stopped = (this->m_renderingStopped && this->m_eventsStopped);
  pthread_mutex_unlock(&dataMutex);
  return stopped;
}

void tgTomGineThread::Snapshot(const char* filename)
{
  if (this->m_renderingStopped)
    return;

  pthread_mutex_lock(&dataMutex);
  m_snapshotFile = std::string(filename);
  m_snapshot = true;
  sem_post(&renderSem);
  pthread_mutex_unlock(&dataMutex);

  sem_wait(&snapshotSem);
}

void tgTomGineThread::StartRecording(std::string path, unsigned fps)
{
  if (this->m_renderingStopped)
    return;
  SetFrameRate(fps);
  pthread_mutex_lock(&dataMutex);
  m_recorderFrame = 0;
  m_recorderPath = path;
  m_record = true;
  sem_post(&renderSem);
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::StopRecording()
{
  if (this->m_renderingStopped)
    return;
  SetFrameRate(0);
  pthread_mutex_lock(&dataMutex);
  m_record = false;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::GetDepthBuffer(cv::Mat1f &z, TomGine::tgCamera &cam)
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  this->m_waitForZBuffer = true;
  sem_post(&renderSem);
  pthread_mutex_unlock(&dataMutex);

  sem_wait(&renderFinishedSem);

  pthread_mutex_lock(&dataMutex);
  this->m_zBuffer.copyTo(z);
  cam = this->m_engine->GetCamera();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::Clear()
{
  if (this->m_renderingStopped)
    return;
  pthread_mutex_lock(&dataMutex);
  m_clear = true;
  sem_post(&renderSem);
  pthread_mutex_unlock(&dataMutex);

  sem_wait(&clearFinishedSem);

  pthread_mutex_lock(&dataMutex);
  m_clear = false;
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearPointClouds()
{
  pthread_mutex_lock(&dataMutex);
  for (unsigned i = 0; i < this->m_pointclouds.size(); i++)
    delete this->m_pointclouds[i];
  m_pointclouds.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearPoints2D()
{
  pthread_mutex_lock(&dataMutex);
  m_points2D.clear();
  m_pointSize2D.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearPoints3D()
{
  pthread_mutex_lock(&dataMutex);
  m_points3D.clear();
  m_pointSize3D.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearLabels()
{
  pthread_mutex_lock(&dataMutex);
  m_labels2D.clear();
  m_labels3D.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearLines2D()
{
  pthread_mutex_lock(&dataMutex);
  m_lines2D.clear();
  m_lineCols2D.clear();
  m_lineWidth2D.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearLines3D()
{
  pthread_mutex_lock(&dataMutex);
  m_lines3D.clear();
  m_lineCols3D.clear();
  m_lineWidth3D.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearModels()
{
  pthread_mutex_lock(&dataMutex);
  m_models2D.clear();
  m_models3D.clear();
  m_modelpointers.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearModels2D()
{
  pthread_mutex_lock(&dataMutex);
  m_models2D.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearModels3D()
{
  pthread_mutex_lock(&dataMutex);
  m_models3D.clear();
  pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::DragDropModels(float speed)
{
  this->StartEventListener(100);
  bool stopDragDrop(false);
  while (!this->Stopped() && !stopDragDrop) {
    int id;
    std::list<TomGine::Event> events;
    this->GetEventQueue(events);
    while (!events.empty()) {

      TomGine::Event ev = events.front();
      events.pop_front();

      if (ev.type == TomGine::TMGL_Press && ev.input == TomGine::TMGL_Return) {
        return;
      }

      if (ev.type == TomGine::TMGL_Release && ev.input == TomGine::TMGL_d) {
        bool return1(false);
        bool return2(false);

        TomGine::Event::Motion motion;
        motion.x = ev.motion.x;
        motion.y = ev.motion.y;
        this->SelectModel(ev.motion.x, ev.motion.y, id);

        // look if a release event is in event que
        while (!events.empty()) {
          ev = events.front();
          events.pop_front();
          if (ev.type == TomGine::TMGL_Release && ev.input == TomGine::TMGL_d)
            return1 = true;
        }

        if (!return1) {
          while (!return2) {

            // move object
            this->GetEventQueue(events);
            while (!events.empty()) {
              ev = events.front();
              events.pop_front();

              if (ev.type == TomGine::TMGL_Release && ev.input == TomGine::TMGL_d)
                return2 = true;

              else if (ev.type == TomGine::TMGL_Motion) {

                motion.x = ev.motion.x - motion.x;
                motion.y = ev.motion.y - motion.y;

                pthread_mutex_lock(&dataMutex);
                m_models3D[id].m_pose.Translate(
                    TomGine::vec3(speed * motion.x, speed * motion.y, 0.0));
                pthread_mutex_unlock(&dataMutex);

                motion.x = ev.motion.x;
                motion.y = ev.motion.y;

              }
            }

          }

        }
      }

    }
  }
  this->StopEventListener();
}

