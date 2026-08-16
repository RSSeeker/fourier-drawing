/* ============================================================================
   傅里叶画板 · Fourier Epicycles  v3
   ----------------------------------------------------------------------------
   纯 Win32 + GDI 实现，无任何第三方库，构建为单个 exe。

   功能：
   1. 左侧画板手绘图案（鼠标绘制、撤销、清空、笔色/粗细、示例图案）。
   2. 离散傅里叶变换：每一笔笔画独立重采样 → DFT → 按模长取前 M 个系数，
      每个系数 = 一个旋转的圆，圆链末端即复现的轨迹；多笔画同时动画。
   3. 轨迹可选「渐隐」效果（从新到旧由亮变暗，每个笔画一种颜色）。
   4. 导出 / 导入圆参数（JSON v2：strokes 数组；兼容 v1 顶层 circles 格式）。
      导出时用系统「另存为」对话框选择保存位置。

   构建：
     gcc -O2 -municode -mwindows fourier-drawing.c -o fourier-drawing.exe \
         -lcomctl32 -lcomdlg32 -lgdi32 -luser32
   自测（无界面，写 selftest.txt 到 exe 同目录）：
     fourier-drawing.exe --selftest
   ========================================================================== */

#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif
#ifndef SS_END_ELLIPSIS
#define SS_END_ELLIPSIS 0x40
#endif
#define WIN32_LEAN_AND_MEAN
#define _WIN32_WINNT 0x0601
#define WINVER 0x0601

#include <windows.h>
#include <commctrl.h>
#include <commdlg.h>
#include <fcntl.h>
#include <io.h>
#include <locale.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <wchar.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef _MSC_VER
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "comdlg32.lib")
#endif

/* ---------------- 常量 ---------------- */
#define CX 600                       /* 画布尺寸 */
#define CY 600
#define MARGIN 12
#define N_SAMP 2048                  /* 每笔画重采样点数 */
#define N_EXT (N_SAMP / 16)          /* 闭合返回段样本数（消除开放笔画的吉布斯振荡） */
#define MAX_PERIOD (N_SAMP + N_EXT)  /* 动画周期上限 */
#define MAX_CIRCLES 400              /* 最大圆数量（每个笔画） */
#define MAX_ANIM_STROKES 32          /* 最多同时动画的笔画数 */
#define FADE_LEVELS 32               /* 轨迹渐隐色阶数 */
#define DRAW_X MARGIN
#define DRAW_Y MARGIN
#define ANIM_X (2 * MARGIN + CX)
#define ANIM_Y MARGIN
#define CLIENT_W (ANIM_X + CX + MARGIN)
#define ROW1_Y (MARGIN + CY + 10)
#define ROW2_Y (ROW1_Y + 32)
#define ROW_H 26
#define STATUS_Y (ROW2_Y + 32)
#define CLIENT_H (STATUS_Y + 20 + MARGIN)

/* 控件 ID */
#define IDC_BTN_COMPUTE 101
#define IDC_BTN_HEART   102
#define IDC_BTN_SPIRAL  103
#define IDC_BTN_UNDO    104
#define IDC_BTN_CLEAR   105
#define IDC_BTN_COLOR   106
#define IDC_BTN_PLAY    107
#define IDC_BTN_RESET   108
#define IDC_TB_CIRCLES  109
#define IDC_TB_SPEED    110
#define IDC_TB_PENSIZE  111
#define IDC_CHK_CIRCLES 112
#define IDC_CHK_SPOKES  113
#define IDC_CHK_TRAIL   114
#define IDC_CHK_REF     115
#define IDC_BTN_EXPORT  116
#define IDC_BTN_IMPORT  117
#define IDC_LBL_PENSIZE 118
#define IDC_LBL_DRAWINFO 119
#define IDC_STATUS      120
#define IDC_CHK_FADE    121

/* ---------------- 基础类型 ---------------- */
typedef struct { double x, y; } Pt;
typedef struct { int k; double re, im, mag; } Coeff;
typedef struct { int k; double re, im, radius, phase; } Circle;
typedef struct { Pt *pts; int n, cap; COLORREF color; int size; } Stroke;
typedef struct { Stroke *s; int n, cap; } StrokeList;

/* 一个笔画对应的动画：独立圆轮 + 路径 + 轨迹 */
typedef struct {
    Coeff full[MAX_PERIOD];         /* 全部系数（闭合路径的 DFT，按模长降序） */
    int fullN;
    Circle circles[MAX_CIRCLES];
    int M;
    Pt path[N_SAMP];                /* 笔画路径（不含返回段，用于参考线 / 导出） */
    int pathN;                      /* = N_stroke */
    int N;                          /* 动画周期（含返回段） */
    int N_stroke;                   /* 笔画段样本数：轨迹只在笔画段记录 */
    double n;                       /* 该笔画的当前相位 */
    Pt trail[N_SAMP + 8];
    int trailN;
} StrokeAnim;

static int coeff_cmp(const void *a, const void *b);
static int circle_cmp(const void *a, const void *b);

static double dist(double x, double y) { return sqrt(x * x + y * y); }
static double clampd(double v, double a, double b) { if (v < a) return a; if (v > b) return b; return v; }
static int    rndi(double v) { return (int)(v >= 0 ? v + 0.5 : v - 0.5); }

/* 每个笔画的配色（黑色动画区上的亮色系，黑底可见） */
static const COLORREF g_palette[8] = {
    RGB(78, 161, 255), RGB(90, 220, 130), RGB(255, 190, 80), RGB(200, 130, 255),
    RGB(80, 220, 220), RGB(255, 140, 160), RGB(190, 220, 90), RGB(255, 160, 70)
};
static COLORREF col_lerp(COLORREF a, COLORREF b, double t) {
    int r = (int)(GetRValue(a) + (GetRValue(b) - GetRValue(a)) * t);
    int g = (int)(GetGValue(a) + (GetGValue(b) - GetGValue(a)) * t);
    int bl = (int)(GetBValue(a) + (GetBValue(b) - GetBValue(a)) * t);
    return RGB(r, g, bl);
}

/* ============================================================================
   纯算法
   ========================================================================== */

/* 按弧长把点列均匀重采样为 N 个点 */
static void resample(const Pt *pts, int n, Pt *out, int N) {
    if (n < 2) { for (int i = 0; i < N; i++) out[i] = pts[0]; return; }
    double *lens = (double *)malloc(sizeof(double) * (size_t)n);
    if (!lens) { for (int i = 0; i < N; i++) out[i] = pts[0]; return; }
    lens[0] = 0;
    for (int i = 1; i < n; i++)
        lens[i] = lens[i - 1] + dist(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    double total = lens[n - 1];
    if (total < 1e-6) { for (int i = 0; i < N; i++) out[i] = pts[0]; free(lens); return; }
    int j = 0;
    for (int i = 0; i < N; i++) {
        double target = total * i / N;
        while (j < n - 2 && lens[j + 1] < target) j++;
        double seg = lens[j + 1] - lens[j];
        if (seg < 1e-9) seg = 1e-9;
        double t = clampd((target - lens[j]) / seg, 0.0, 1.0);
        out[i].x = pts[j].x + (pts[j + 1].x - pts[j].x) * t;
        out[i].y = pts[j].y + (pts[j + 1].y - pts[j].y) * t;
    }
    free(lens);
}

/* DFT：X[k]=(1/N)Σ x[n]·e^{-2πikn/N}，k 归一到 [-N/2,N/2)，按模长降序返回 */
static void dft(const Pt *pts, int N, Coeff *out) {
    for (int k = 0; k < N; k++) {
        double w = 2.0 * M_PI * k / N;
        double re = 0, im = 0;
        double c = 1.0, s = 0.0;      /* (c,s)=e^{iwn} 旋转递推 */
        double cw = cos(w), sw = sin(w);
        for (int n = 0; n < N; n++) {
            double x = pts[n].x, y = pts[n].y;
            re += x * c + y * s;
            im += y * c - x * s;
            double nc = c * cw - s * sw;
            double ns = c * sw + s * cw;
            c = nc; s = ns;
        }
        re /= N; im /= N;
        out[k].k = (k <= N / 2) ? k : k - N;
        out[k].re = re; out[k].im = im; out[k].mag = dist(re, im);
    }
    qsort(out, (size_t)N, sizeof(Coeff), coeff_cmp);
}

static int coeff_cmp(const void *a, const void *b) {
    const Coeff *x = (const Coeff *)a, *y = (const Coeff *)b;
    if (y->mag > x->mag) return 1;
    if (y->mag < x->mag) return -1;
    return abs(x->k) - abs(y->k);
}
static int circle_cmp(const void *a, const void *b) {
    const Circle *x = (const Circle *)a, *y = (const Circle *)b;
    if (y->radius > x->radius) return 1;
    if (y->radius < x->radius) return -1;
    return abs(x->k) - abs(y->k);
}

/* 圆链：out[i] = 前 i+1 个圆的向量和（第 i 个圆的末端），最后一个即画笔位置 */
static void chain_at(const Circle *cs, int M, double n, int N, Pt *out) {
    double x = 0, y = 0;
    for (int i = 0; i < M; i++) {
        double a = cs[i].phase + 2.0 * M_PI * cs[i].k * n / N;
        x += cs[i].radius * cos(a);
        y += cs[i].radius * sin(a);
        out[i].x = x; out[i].y = y;
    }
}

/* 平均重建误差（像素）：在 path[0..pathN) 上评估，重建周期为 period */
static double approx_error(const Pt *path, int pathN, const Circle *cs, int M, int period) {
    double sum = 0;
    for (int i = 0; i < pathN; i++) {
        double th0 = 2.0 * M_PI * i / period, x = 0, y = 0;
        for (int j = 0; j < M; j++) {
            double a = cs[j].phase + cs[j].k * th0;
            x += cs[j].radius * cos(a);
            y += cs[j].radius * sin(a);
        }
        sum += dist(x - path[i].x, y - path[i].y);
    }
    return sum / pathN;
}

/* ============================================================================
   极简 JSON（解析 + 写出）
   ========================================================================== */
typedef enum { JT_NULL, JT_BOOL, JT_NUM, JT_STR, JT_ARR, JT_OBJ } JType;
typedef struct JV JV;
typedef struct JPair JPair;
struct JPair { wchar_t *key; JV *val; };
struct JV {
    JType type;
    double num; int b;
    wchar_t *str;
    JV *items; int n, cap;      /* 数组 */
    JPair *pairs; int pn, pcap; /* 对象 */
};

typedef struct AB { void *p; struct AB *next; } AB;
static AB *g_arena = NULL;
static void *a_malloc(size_t sz) {
    void *p = malloc(sz);
    if (!p) return NULL;
    AB *b = (AB *)malloc(sizeof(AB));
    if (!b) { free(p); return NULL; }
    b->p = p; b->next = g_arena; g_arena = b;
    return p;
}
static void a_free_all(void) {
    AB *b = g_arena;
    while (b) { AB *nx = b->next; free(b->p); free(b); b = nx; }
    g_arena = NULL;
}

typedef struct { char *b; int len, cap; } Sbuf;
static void sb_putc(Sbuf *sb, char c) {
    if (sb->len + 2 >= sb->cap) {
        int nc = sb->cap ? sb->cap * 2 : 128;
        char *nb = (char *)a_malloc((size_t)nc);
        if (!nb) return;
        if (sb->b) memcpy(nb, sb->b, (size_t)sb->len);
        sb->b = nb; sb->cap = nc;
    }
    sb->b[sb->len++] = c;
}

static void utf8_encode(unsigned int cp, char *out, int *len) {
    if (cp < 0x80)       { out[0] = (char)cp; *len = 1; }
    else if (cp < 0x800) { out[0] = (char)(0xC0 | (cp >> 6)); out[1] = (char)(0x80 | (cp & 0x3F)); *len = 2; }
    else if (cp < 0x10000) { out[0] = (char)(0xE0 | (cp >> 12)); out[1] = (char)(0x80 | ((cp >> 6) & 0x3F)); out[2] = (char)(0x80 | (cp & 0x3F)); *len = 3; }
    else { out[0] = (char)(0xF0 | (cp >> 18)); out[1] = (char)(0x80 | ((cp >> 12) & 0x3F)); out[2] = (char)(0x80 | ((cp >> 6) & 0x3F)); out[3] = (char)(0x80 | (cp & 0x3F)); *len = 4; }
}

static int hex4(const char *s, unsigned int *out) {
    unsigned int v = 0;
    for (int i = 0; i < 4; i++) {
        char c = s[i]; unsigned int d;
        if (c >= '0' && c <= '9') d = (unsigned int)(c - '0');
        else if (c >= 'a' && c <= 'f') d = (unsigned int)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') d = (unsigned int)(c - 'A' + 10);
        else return 0;
        v = v * 16 + d;
    }
    *out = v; return 1;
}

static const char *js_ws(const char *s) { while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') s++; return s; }
static const char *js_value(const char *s, JV *v);

static const char *js_parse_string(const char *s, wchar_t **out) {
    s++; /* '"' */
    Sbuf sb = { NULL, 0, 0 };
    while (*s && *s != '"') {
        if (*s == '\\') {
            s++;
            char c = *s; s++;
            switch (c) {
                case '"':  sb_putc(&sb, '"'); break;
                case '\\': sb_putc(&sb, '\\'); break;
                case '/':  sb_putc(&sb, '/'); break;
                case 'b':  sb_putc(&sb, '\b'); break;
                case 'f':  sb_putc(&sb, '\f'); break;
                case 'n':  sb_putc(&sb, '\n'); break;
                case 'r':  sb_putc(&sb, '\r'); break;
                case 't':  sb_putc(&sb, '\t'); break;
                case 'u': {
                    unsigned int cp;
                    if (!hex4(s, &cp)) return NULL;
                    s += 4;
                    if (cp >= 0xD800 && cp <= 0xDBFF && s[0] == '\\' && s[1] == 'u') {
                        unsigned int lo;
                        if (hex4(s + 2, &lo) && lo >= 0xDC00 && lo <= 0xDFFF) {
                            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                            s += 6;
                        }
                    }
                    char e[4]; int el;
                    utf8_encode(cp, e, &el);
                    for (int i = 0; i < el; i++) sb_putc(&sb, e[i]);
                    break;
                }
                default: return NULL;
            }
        } else {
            sb_putc(&sb, *s++);
        }
    }
    if (*s != '"') return NULL;
    s++;
    if (sb.len == 0) {
        wchar_t *ws = (wchar_t *)a_malloc(sizeof(wchar_t));
        if (!ws) return NULL;
        ws[0] = 0;
        *out = ws;
        return s;
    }
    int wlen = MultiByteToWideChar(CP_UTF8, 0, sb.b, sb.len, NULL, 0);
    if (wlen <= 0) return NULL;
    wchar_t *ws = (wchar_t *)a_malloc(sizeof(wchar_t) * ((size_t)wlen + 1));
    if (!ws) return NULL;
    MultiByteToWideChar(CP_UTF8, 0, sb.b, sb.len, ws, wlen);
    ws[wlen] = 0;
    *out = ws;
    return s;
}

static const char *js_array(const char *s, JV *v) {
    s++; v->type = JT_ARR;
    s = js_ws(s);
    if (*s == ']') return s + 1;
    for (;;) {
        if (v->n >= v->cap) {
            v->cap = v->cap ? v->cap * 2 : 8;
            JV *ni = (JV *)a_malloc(sizeof(JV) * (size_t)v->cap);
            if (!ni) return NULL;
            memset(ni, 0, sizeof(JV) * (size_t)v->cap); /* 关键：零化，防止读取未初始化字段 */
            if (v->items) memcpy(ni, v->items, sizeof(JV) * (size_t)v->n);
            v->items = ni;
        }
        s = js_value(s, &v->items[v->n]);
        if (!s) return NULL;
        v->n++;
        s = js_ws(s);
        if (*s == ',') { s++; continue; }
        if (*s == ']') return s + 1;
        return NULL;
    }
}

static const char *js_object(const char *s, JV *v) {
    s++; v->type = JT_OBJ;
    s = js_ws(s);
    if (*s == '}') return s + 1;
    for (;;) {
        s = js_ws(s);
        if (*s != '"') return NULL;
        wchar_t *key;
        s = js_parse_string(s, &key);
        if (!s) return NULL;
        s = js_ws(s);
        if (*s != ':') return NULL;
        s++;
        if (v->pn >= v->pcap) {
            v->pcap = v->pcap ? v->pcap * 2 : 8;
            JPair *np = (JPair *)a_malloc(sizeof(JPair) * (size_t)v->pcap);
            if (!np) return NULL;
            memset(np, 0, sizeof(JPair) * (size_t)v->pcap); /* 零化，避免脏数据 */
            if (v->pairs) memcpy(np, v->pairs, sizeof(JPair) * (size_t)v->pn);
            v->pairs = np;
        }
        v->pairs[v->pn].key = key;
        v->pairs[v->pn].val = (JV *)a_malloc(sizeof(JV));
        if (!v->pairs[v->pn].val) return NULL;
        memset(v->pairs[v->pn].val, 0, sizeof(JV));
        s = js_value(s, v->pairs[v->pn].val);
        if (!s) return NULL;
        v->pn++;
        s = js_ws(s);
        if (*s == ',') { s++; continue; }
        if (*s == '}') return s + 1;
        return NULL;
    }
}

static const char *js_value(const char *s, JV *v) {
    s = js_ws(s);
    if (!*s) return NULL;
    if (*s == '{') return js_object(s, v);
    if (*s == '[') return js_array(s, v);
    if (*s == '"') { v->type = JT_STR; return js_parse_string(s, &v->str); }
    if (*s == 't') { if (!strncmp(s, "true", 4)) { v->type = JT_BOOL; v->b = 1; return s + 4; } return NULL; }
    if (*s == 'f') { if (!strncmp(s, "false", 5)) { v->type = JT_BOOL; v->b = 0; return s + 5; } return NULL; }
    if (*s == 'n') { if (!strncmp(s, "null", 4)) { v->type = JT_NULL; return s + 4; } return NULL; }
    if (*s == '-' || (*s >= '0' && *s <= '9')) {
        char *end;
        v->type = JT_NUM;
        v->num = strtod(s, &end);
        if (end == s) return NULL;
        return end;
    }
    return NULL;
}

static JV *json_parse(const char *bytes) {
    JV root; memset(&root, 0, sizeof(root));
    const char *s = js_value(bytes, &root);
    if (!s) return NULL;
    s = js_ws(s);
    if (*s) return NULL;
    JV *r = (JV *)a_malloc(sizeof(JV));
    if (!r) return NULL;
    memcpy(r, &root, sizeof(JV));
    return r;
}

static JV *jv_find(const JV *obj, const char *key8) {
    if (!obj || obj->type != JT_OBJ) return NULL;
    wchar_t wk[64];
    MultiByteToWideChar(CP_UTF8, 0, key8, -1, wk, 64);
    for (int i = 0; i < obj->pn; i++)
        if (wcscmp(obj->pairs[i].key, wk) == 0) return obj->pairs[i].val;
    return NULL;
}
static double jv_num(const JV *v, double def) { return (v && v->type == JT_NUM) ? v->num : def; }

/* 写出 v2 JSON（多笔画） */
static void json_write_strokes(FILE *f, const StrokeAnim *anims, int count) {
    fputs("{\n  \"app\":\"fourier-drawing\",\n  \"version\":2,\n"
          "  \"description\":\"Fourier drawing epicycles. strokes[i]={sampleCount, circles:[{k,radius,phase,re,im}], path:[[x,y]]}; k=frequency(neg=clockwise)\",\n"
          "  \"strokes\":[\n", f);
    for (int si = 0; si < count; si++) {
        const StrokeAnim *a = &anims[si];
        fputs("    {\n      \"sampleCount\":", f);
        fprintf(f, "%d", a->N);      /* 动画周期（含闭合返回段） */
        fputs(",\n      \"strokeCount\":", f);
        fprintf(f, "%d", a->N_stroke > 0 ? a->N_stroke : a->N); /* 笔画段样本数 */
        fputs(",\n      \"circles\":[\n", f);
        for (int i = 0; i < a->M; i++)
            fprintf(f, "        {\"k\":%d,\"radius\":%.6f,\"phase\":%.6f,\"re\":%.6f,\"im\":%.6f}%s\n",
                    a->circles[i].k, a->circles[i].radius, a->circles[i].phase,
                    a->circles[i].re, a->circles[i].im, i < a->M - 1 ? "," : "");
        fputs("      ],\n      \"path\":[\n", f);
        for (int i = 0; i < a->pathN; i++)
            fprintf(f, "        [%.2f,%.2f]%s\n", a->path[i].x, a->path[i].y,
                    i < a->pathN - 1 ? "," : "");
        fputs("      ]\n    }", f);
        fputs(si < count - 1 ? ",\n" : "\n", f);
    }
    fputs("  ]\n}\n", f);
}

/* ============================================================================
   全局状态
   ========================================================================== */
static HWND g_hwnd = NULL;
static StrokeList g_strokes = { NULL, 0, 0 };
static Pt *g_cur = NULL; static int g_curN = 0, g_curCap = 0;
static int g_drawing = 0;
static COLORREF g_penColor = RGB(17, 24, 39);
static int g_penSize = 4;
static COLORREF g_custom[16] = { 0 };

static StrokeAnim g_anims[MAX_ANIM_STROKES];
static StrokeAnim g_scratch[MAX_ANIM_STROKES];  /* 导入解析暂存（大结构，避免栈溢出） */
static int g_animCount = 0;   /* 0 = 无动画 */
static int g_playing = 1;
static double g_speed = 0.35;
static int g_circleCount = 120;

/* GDI */
static HDC g_drawMem = NULL, g_animMem = NULL;
static HBITMAP g_drawBmp = NULL, g_animBmp = NULL;
static HDC g_cliMem = NULL;   /* 整窗合成缓冲：背景+画布+动画+边框全部合成后一次 BitBlt，
                                 窗口表面每次绘制只更新一次，杜绝中间态闪烁 */
static HBITMAP g_cliBmp = NULL;
static HFONT g_font = NULL;
static RECT g_drawRect, g_animRect;

/* 缓存的画笔 / 画刷 */
static HPEN g_penCircle = NULL, g_penSpoke = NULL, g_penRef = NULL, g_penTipOuter = NULL;
static HPEN g_penStroke[8];              /* 各笔画实色轨迹 */
static HBRUSH g_brushPanel = NULL, g_brushTip = NULL;
static HBRUSH g_brushStroke[8];          /* 各笔画笔尖填充 */
static HPEN g_fadePens[8][FADE_LEVELS];  /* 渐隐色阶 */

/* 控件句柄 */
static HWND g_btnCompute, g_btnHeart, g_btnSpiral, g_btnUndo, g_btnClear, g_btnColor;
static HWND g_btnPlay, g_btnReset, g_tbCircles, g_tbSpeed, g_tbPenSize;
static HWND g_lblCircles, g_lblSpeed, g_lblPenSize, g_lblDrawInfo, g_status;
static HWND g_btnExport, g_btnImport;

static void set_status(const wchar_t *txt) { SetWindowTextW(g_status, txt); }
static void set_play_label(void) { SetWindowTextW(g_btnPlay, g_playing ? L"暂停" : L"播放"); }

static void update_draw_info(void) {
    int len = 0;
    for (int i = 0; i < g_strokes.n; i++) len += g_strokes.s[i].n;
    wchar_t buf[64];
    swprintf(buf, 64, L"笔画 %d · 点 %d", g_strokes.n, len);
    SetWindowTextW(g_lblDrawInfo, buf);
}

/* ---------------- 绘制画板相关 ---------------- */
static void stroke_dot(double x, double y, int size, COLORREF col) {
    HBRUSH b = CreateSolidBrush(col);
    HGDIOBJ ob = SelectObject(g_drawMem, b);
    Ellipse(g_drawMem, (int)(x - size / 2.0), (int)(y - size / 2.0),
            (int)(x + size / 2.0 + 0.5), (int)(y + size / 2.0 + 0.5));
    SelectObject(g_drawMem, ob);
    DeleteObject(b);
}
static void stroke_line(Pt a, Pt b, int size, COLORREF col) {
    HPEN p = CreatePen(PS_SOLID, size, col);
    HGDIOBJ op = SelectObject(g_drawMem, p);
    MoveToEx(g_drawMem, (int)a.x, (int)a.y, NULL);
    LineTo(g_drawMem, (int)b.x, (int)b.y);
    SelectObject(g_drawMem, op);
    DeleteObject(p);
    stroke_dot(b.x, b.y, size, col); /* 圆头衔接 */
}
static void redraw_all_strokes(void) {
    RECT r = { 0, 0, CX, CY };
    FillRect(g_drawMem, &r, (HBRUSH)GetStockObject(WHITE_BRUSH));
    for (int i = 0; i < g_strokes.n; i++) {
        Stroke *st = &g_strokes.s[i];
        if (st->n <= 0) continue;
        stroke_dot(st->pts[0].x, st->pts[0].y, st->size, st->color);
        for (int j = 1; j < st->n; j++)
            stroke_line(st->pts[j - 1], st->pts[j], st->size, st->color);
    }
    update_draw_info();
}

static void strokes_push(const Stroke *st) {
    if (g_strokes.n >= g_strokes.cap) {
        g_strokes.cap = g_strokes.cap ? g_strokes.cap * 2 : 8;
        g_strokes.s = (Stroke *)realloc(g_strokes.s, sizeof(Stroke) * (size_t)g_strokes.cap);
    }
    g_strokes.s[g_strokes.n++] = *st;
}
static void cur_push(Pt p) {
    if (g_curN >= g_curCap) {
        g_curCap = g_curCap ? g_curCap * 2 : 256;
        g_cur = (Pt *)realloc(g_cur, sizeof(Pt) * (size_t)g_curCap);
    }
    g_cur[g_curN++] = p;
}

/* ---------------- 傅里叶变换 ---------------- */

/* 按当前 g_circleCount 从每个动画的 full 系数切片重建 circles */
static void rebuild_stroke_anims(void) {
    for (int si = 0; si < g_animCount; si++) {
        StrokeAnim *a = &g_anims[si];
        int m = g_circleCount;
        if (m > a->fullN) m = a->fullN;
        if (m > MAX_CIRCLES) m = MAX_CIRCLES;
        if (m < 0) m = 0;
        a->M = m;
        for (int i = 0; i < m; i++) {
            a->circles[i].k = a->full[i].k;
            a->circles[i].re = a->full[i].re;
            a->circles[i].im = a->full[i].im;
            a->circles[i].radius = a->full[i].mag;
            a->circles[i].phase = atan2(a->full[i].im, a->full[i].re);
        }
        a->trailN = 0;
    }
}

/* 把一个笔画构建为动画：重采样 → 闭合（追加从末尾回起点的返回段）→ DFT。
   闭合消除了开放笔画周期延拓的跳变，从而消除吉布斯振荡产生的异常直线。 */
static void build_anim(StrokeAnim *a, const Pt *pts, int n, int M) {
    const int N = N_SAMP;
    const int N_total = MAX_PERIOD;
    if (M > MAX_CIRCLES) M = MAX_CIRCLES;
    resample(pts, n, a->path, N);
    a->pathN = N;
    a->N_stroke = N;
    a->N = N_total;
    Pt ext[MAX_PERIOD];
    memcpy(ext, a->path, sizeof(Pt) * (size_t)N);
    Pt p0 = a->path[0], p1 = a->path[N - 1];
    for (int i = 0; i < N_EXT; i++) {
        double t = (double)(i + 1) / (N_EXT + 1);
        ext[N + i].x = p1.x + (p0.x - p1.x) * t;
        ext[N + i].y = p1.y + (p0.y - p1.y) * t;
    }
    dft(ext, N_total, a->full);
    a->fullN = N_total;
    a->M = M;
    a->n = 0;
    a->trailN = 0;
    for (int i = 0; i < M; i++) {
        a->circles[i].k = a->full[i].k;
        a->circles[i].re = a->full[i].re;
        a->circles[i].im = a->full[i].im;
        a->circles[i].radius = a->full[i].mag;
        a->circles[i].phase = atan2(a->full[i].im, a->full[i].re);
    }
}

/* 每个笔画独立生成自己的圆轮 */
static void compute(HWND h) {
    if (g_strokes.n == 0) {
        MessageBoxW(h, L"请先在左侧画板绘制图案", L"傅里叶画板", MB_ICONINFORMATION);
        return;
    }
    int animCount = 0;
    for (int si = 0; si < g_strokes.n && animCount < MAX_ANIM_STROKES; si++) {
        Stroke *st = &g_strokes.s[si];
        if (st->n < 2) continue;  /* 单点笔画跳过 */
        build_anim(&g_anims[animCount], st->pts, st->n, g_circleCount);
        animCount++;
    }
    if (animCount == 0) {
        MessageBoxW(h, L"笔画点数不足，无法生成动画", L"傅里叶画板", MB_ICONINFORMATION);
        return;
    }
    /* 必须先更新数量：rebuild_stroke_anims 按 g_animCount 遍历，
       否则清空后重建循环不会执行，circles 残留上一次的旧数据 */
    g_animCount = animCount;
    rebuild_stroke_anims();
    g_playing = 1;
    set_play_label();
    double errSum = 0;
    for (int si = 0; si < g_animCount; si++)
        errSum += approx_error(g_anims[si].path, g_anims[si].N_stroke,
                               g_anims[si].circles, g_anims[si].M, g_anims[si].N);
    double err = errSum / g_animCount;
    wchar_t buf[256];
    swprintf(buf, 256, L"%d 个笔画 × %d 个圆 · 平均误差 %.1f px（画布 %.2f%%）",
             g_animCount, g_circleCount, err, err / CX * 100.0);
    set_status(buf);
    InvalidateRect(h, NULL, FALSE);
}

/* ---------------- 示例图案 ---------------- */
static void fit_pts(Pt *p, int n, double target) {
    double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    for (int i = 0; i < n; i++) {
        if (p[i].x < minx) minx = p[i].x;
        if (p[i].y < miny) miny = p[i].y;
        if (p[i].x > maxx) maxx = p[i].x;
        if (p[i].y > maxy) maxy = p[i].y;
    }
    double w = maxx - minx, h = maxy - miny;
    if (w < 1) w = 1;
    if (h < 1) h = 1;
    double s = (target / w < target / h) ? target / w : target / h;
    double cx = (maxx + minx) / 2, cy = (maxy + miny) / 2;
    for (int i = 0; i < n; i++) {
        double nx = CX / 2.0 + (p[i].x - cx) * s;
        double ny = CY / 2.0 - (p[i].y - cy) * s;
        p[i].x = nx; p[i].y = ny;
    }
}
static Pt *demo_heart(int *n) {
    *n = 600;
    Pt *p = (Pt *)malloc(sizeof(Pt) * 600);
    if (!p) { *n = 0; return NULL; }
    for (int i = 0; i < 600; i++) {
        double t = 2.0 * M_PI * i / 600;
        double s = sin(t);
        p[i].x = 16.0 * s * s * s;
        p[i].y = 13.0 * cos(t) - 5.0 * cos(2 * t) - 2.0 * cos(3 * t) - cos(4 * t);
    }
    fit_pts(p, 600, 480);
    return p;
}
static Pt *demo_spiral(int *n) {
    *n = 1400;
    Pt *p = (Pt *)malloc(sizeof(Pt) * 1400);
    if (!p) { *n = 0; return NULL; }
    for (int i = 0; i < 1400; i++) {
        double t = 3.0 * 2.0 * M_PI * i / (1400 - 1);
        double r = 6.0 + 170.0 * t / (3.0 * 2.0 * M_PI);
        p[i].x = r * cos(t);
        p[i].y = r * sin(t);
    }
    fit_pts(p, 1400, 520);
    return p;
}
static void load_demo(HWND h, Pt *pts, int n) {
    if (!pts) return;
    for (int i = 0; i < g_strokes.n; i++) free(g_strokes.s[i].pts);
    g_strokes.n = 0;
    if (g_strokes.cap < 1) { g_strokes.cap = 8; g_strokes.s = (Stroke *)malloc(sizeof(Stroke) * 8); }
    Stroke st;
    st.pts = pts; st.n = n; st.cap = n; st.color = RGB(17, 24, 39); st.size = 4;
    g_strokes.s[0] = st;
    g_strokes.n = 1;
    redraw_all_strokes();
    compute(h);
}

/* ---------------- 撤销 / 清空 ---------------- */
static void undo_stroke(void) {
    if (g_strokes.n > 0) {
        free(g_strokes.s[g_strokes.n - 1].pts);
        g_strokes.n--;
        redraw_all_strokes();
        InvalidateRect(g_hwnd, &g_drawRect, FALSE);
    }
}
static void clear_all(HWND h) {
    for (int i = 0; i < g_strokes.n; i++) free(g_strokes.s[i].pts);
    g_strokes.n = 0;
    if (g_cur) { free(g_cur); g_cur = NULL; g_curN = 0; g_curCap = 0; }
    g_animCount = 0;
    redraw_all_strokes();
    set_status(L"画板已清空");
    InvalidateRect(h, NULL, FALSE);
}

/* ---------------- 导出（另存为对话框） ---------------- */
static void do_export(HWND h) {
    if (g_animCount == 0) {
        MessageBoxW(h, L"当前没有可导出的圆参数（请先生成傅里叶动画）", L"傅里叶画板", MB_ICONINFORMATION);
        return;
    }
    wchar_t filebuf[MAX_PATH + 64] = { 0 };
    wchar_t dir[MAX_PATH];
    GetModuleFileNameW(NULL, dir, MAX_PATH);
    wchar_t *s = wcsrchr(dir, L'\\');
    if (s) *s = 0;
    time_t t = time(NULL);
    struct tm *lt = localtime(&t);
    swprintf(filebuf, MAX_PATH + 64, L"%ls\\fourier-circles-%04d%02d%02d-%02d%02d.json",
             dir, lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday, lt->tm_hour, lt->tm_min);
    OPENFILENAMEW ofn;
    memset(&ofn, 0, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = h;
    ofn.lpstrFilter = L"JSON 圆参数 (*.json)\0*.json\0所有文件 (*.*)\0*.*\0\0";
    ofn.lpstrFile = filebuf;
    ofn.nMaxFile = MAX_PATH + 64;
    ofn.lpstrInitialDir = dir;
    ofn.lpstrDefExt = L"json";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
    if (!GetSaveFileNameW(&ofn)) return;
    FILE *f = _wfopen(ofn.lpstrFile, L"wb");
    if (!f) { MessageBoxW(h, L"无法写入文件", L"错误", MB_ICONERROR); return; }
    json_write_strokes(f, g_anims, g_animCount);
    fclose(f);
    wchar_t msg[512];
    swprintf(msg, 512, L"圆参数已导出（%d 个笔画）：%ls", g_animCount, ofn.lpstrFile);
    set_status(msg);
}

/* ---------------- 导入 ---------------- */

/* 解析一个 stroke 对象（circles + path + sampleCount）填入 StrokeAnim */
static int parse_stroke_obj(const JV *obj, StrokeAnim *a, wchar_t *err, int errcap) {
    memset(a, 0, sizeof(*a));
    JV *carr = jv_find(obj, "circles");
    if (!carr) carr = jv_find(obj, "coefficients");
    if (!carr) carr = jv_find(obj, "epicycles");
    if (!carr || carr->type != JT_ARR || carr->n == 0) {
        swprintf(err, (size_t)errcap, L"笔画缺少 circles 数组");
        return 0;
    }
    int cap = carr->n; if (cap > N_SAMP) cap = N_SAMP;
    Coeff *tmp = (Coeff *)malloc(sizeof(Coeff) * (size_t)cap);
    if (!tmp) { swprintf(err, (size_t)errcap, L"内存不足"); return 0; }
    int tn = 0;
    for (int i = 0; i < carr->n && tn < cap; i++) {
        JV *o = &carr->items[i];
        if (o->type != JT_OBJ) continue;
        double re = 0, im = 0; int hasRC = 0, hasIC = 0;
        JV *jre = jv_find(o, "re"), *jim = jv_find(o, "im");
        if (jre && jre->type == JT_NUM) { re = jre->num; hasRC = 1; }
        if (jim && jim->type == JT_NUM) { im = jim->num; hasIC = 1; }
        if (!(hasRC && hasIC)) {
            double r = jv_num(jv_find(o, "radius"), jv_num(jv_find(o, "mag"), 0));
            double ph = jv_num(jv_find(o, "phase"), 0);
            re = r * cos(ph); im = r * sin(ph);
        }
        double k = jv_num(jv_find(o, "k"), jv_num(jv_find(o, "freq"), 0));
        tmp[tn].k = rndi(k);
        tmp[tn].re = re; tmp[tn].im = im; tmp[tn].mag = dist(re, im);
        tn++;
    }
    if (tn == 0) {
        free(tmp);
        swprintf(err, (size_t)errcap, L"circles 数组为空或格式不正确");
        return 0;
    }
    qsort(tmp, (size_t)tn, sizeof(Coeff), coeff_cmp);
    int N = rndi(jv_num(jv_find(obj, "sampleCount"), 0));
    if (N < 16) N = 16;
    if (N > MAX_PERIOD) N = MAX_PERIOD;
    int N_stroke = rndi(jv_num(jv_find(obj, "strokeCount"), 0));
    /* 路径（可选） */
    JV *pth = jv_find(obj, "path");
    Pt *raw = NULL; int rn = 0;
    if (pth && pth->type == JT_ARR && pth->n > 1) {
        raw = (Pt *)malloc(sizeof(Pt) * (size_t)pth->n);
        if (raw) {
            for (int i = 0; i < pth->n; i++) {
                JV *it = &pth->items[i];
                if (it->type == JT_ARR && it->n >= 2 &&
                    it->items[0].type == JT_NUM && it->items[1].type == JT_NUM) {
                    raw[rn].x = it->items[0].num; raw[rn].y = it->items[1].num; rn++;
                } else if (it->type == JT_OBJ) {
                    raw[rn].x = jv_num(jv_find(it, "x"), 0);
                    raw[rn].y = jv_num(jv_find(it, "y"), 0);
                    rn++;
                }
            }
        }
    }
    if (N_stroke == 0) {
        /* 旧格式（无 strokeCount）：sampleCount 即笔画样本数。
           若有路径 → 重新闭合 + DFT（与生成一致，消除吉布斯振荡）；
           否则直接使用文件系数。 */
        if (rn > 1) {
            N_stroke = N;
            if (N_stroke > N_SAMP) N_stroke = N_SAMP;
            resample(raw, rn, a->path, N_stroke);
            a->pathN = N_stroke;
            int N_total = N_stroke + N_EXT;
            if (N_total > MAX_PERIOD) N_total = MAX_PERIOD;
            Pt ext[MAX_PERIOD];
            memcpy(ext, a->path, sizeof(Pt) * (size_t)N_stroke);
            Pt p0 = a->path[0], p1 = a->path[N_stroke - 1];
            for (int i = 0; i < N_EXT && N_stroke + i < N_total; i++) {
                double t = (double)(i + 1) / (N_EXT + 1);
                ext[N_stroke + i].x = p1.x + (p0.x - p1.x) * t;
                ext[N_stroke + i].y = p1.y + (p0.y - p1.y) * t;
            }
            dft(ext, N_total, a->full);
            a->fullN = N_total;
            a->N = N_total;
            a->N_stroke = N_stroke;
        } else {
            a->N = N;
            a->N_stroke = N;
            a->fullN = tn;
            memcpy(a->full, tmp, sizeof(Coeff) * (size_t)tn);
        }
    } else {
        /* 新格式：直接用文件系数（已含闭合），周期/笔画段来自文件 */
        if (N_stroke > N_SAMP) N_stroke = N_SAMP;
        a->N = N;
        a->N_stroke = N_stroke;
        a->fullN = tn;
        memcpy(a->full, tmp, sizeof(Coeff) * (size_t)tn);
        if (rn > 1) {
            resample(raw, rn, a->path, N_stroke);
            a->pathN = N_stroke;
        }
    }
    free(raw);
    free(tmp);
    a->M = tn; if (a->M > MAX_CIRCLES) a->M = MAX_CIRCLES;
    for (int i = 0; i < a->M; i++) {
        a->circles[i].k = a->full[i].k;
        a->circles[i].re = a->full[i].re;
        a->circles[i].im = a->full[i].im;
        a->circles[i].radius = a->full[i].mag;
        a->circles[i].phase = atan2(a->full[i].im, a->full[i].re);
    }
    a->n = 0;
    return 1;
}

/* 纯数据导入（无 GUI 依赖，可自测）：解析 bytes 填充 g_anims */
static int import_data(const char *bytes, wchar_t *err, int errcap) {
    a_free_all();
    JV *root = json_parse(bytes);
    if (!root) { swprintf(err, (size_t)errcap, L"JSON 语法错误"); return 0; }
    int count = 0;
    JV *strokes = jv_find(root, "strokes");
    if (strokes && strokes->type == JT_ARR) {
        for (int i = 0; i < strokes->n && count < MAX_ANIM_STROKES; i++) {
            if (strokes->items[i].type != JT_OBJ) continue;
            if (!parse_stroke_obj(&strokes->items[i], &g_scratch[count], err, errcap)) return 0;
            count++;
        }
    } else {
        /* v1 兼容：顶层 circles / path / sampleCount */
        if (!parse_stroke_obj(root, &g_scratch[0], err, errcap)) return 0;
        count = 1;
    }
    if (count == 0) { swprintf(err, (size_t)errcap, L"文件不含任何笔画"); return 0; }
    memcpy(g_anims, g_scratch, sizeof(StrokeAnim) * (size_t)count);
    g_animCount = count;
    g_playing = 1;
    return 1;
}

static void do_import(HWND h) {
    wchar_t dir[MAX_PATH];
    GetModuleFileNameW(NULL, dir, MAX_PATH);
    wchar_t *s = wcsrchr(dir, L'\\');
    if (s) *s = 0;
    wchar_t filebuf[MAX_PATH] = { 0 };
    OPENFILENAMEW ofn;
    memset(&ofn, 0, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = h;
    ofn.lpstrFilter = L"JSON 圆参数 (*.json)\0*.json\0所有文件 (*.*)\0*.*\0\0";
    ofn.lpstrFile = filebuf;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrInitialDir = dir;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
    if (!GetOpenFileNameW(&ofn)) return;
    FILE *f = _wfopen(ofn.lpstrFile, L"rb");
    if (!f) { MessageBoxW(h, L"无法打开文件", L"错误", MB_ICONERROR); return; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 20 * 1024 * 1024) {
        fclose(f);
        MessageBoxW(h, L"文件大小异常", L"错误", MB_ICONERROR);
        return;
    }
    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); MessageBoxW(h, L"内存不足", L"错误", MB_ICONERROR); return; }
    fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[sz] = 0;
    wchar_t errmsg[256] = { 0 };
    if (!import_data(buf, errmsg, 256)) {
        MessageBoxW(h, errmsg[0] ? errmsg : L"无法解析该 JSON 文件", L"导入失败", MB_ICONERROR);
        free(buf);
        a_free_all();
        return;
    }
    /* 同步画板：把每个笔画的路径画回左侧 */
    for (int i = 0; i < g_strokes.n; i++) free(g_strokes.s[i].pts);
    g_strokes.n = 0;
    for (int i = 0; i < g_animCount; i++) {
        StrokeAnim *a = &g_anims[i];
        if (a->pathN <= 0) continue;
        Stroke st;
        st.pts = (Pt *)malloc(sizeof(Pt) * (size_t)a->pathN);
        if (!st.pts) continue;
        memcpy(st.pts, a->path, sizeof(Pt) * (size_t)a->pathN);
        st.n = a->pathN; st.cap = a->pathN; st.color = RGB(17, 24, 39); st.size = 4;
        strokes_push(&st);
    }
    redraw_all_strokes();
    set_play_label();
    int m0 = g_animCount > 0 ? g_anims[0].M : 0;
    wchar_t msg[160];
    swprintf(msg, 160, L"已载入 %d 个笔画（每笔 %d 个圆），动画已开始", g_animCount, m0);
    set_status(msg);
    InvalidateRect(h, NULL, FALSE);
    free(buf);
    a_free_all();
}

/* ---------------- 动画渲染（多笔画 + 渐隐） ---------------- */
static void render_animation(HDC dc, HWND h) {
    RECT r = { 0, 0, CX, CY };
    FillRect(dc, &r, (HBRUSH)GetStockObject(BLACK_BRUSH));
    if (g_animCount <= 0) {
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, RGB(140, 150, 170));
        HGDIOBJ of = SelectObject(dc, g_font);
        RECT tr = { 0, CY / 2 - 40, CX, CY / 2 };
        DrawTextW(dc, L"暂无数据", -1, &tr, DT_CENTER | DT_SINGLELINE | DT_VCENTER);
        RECT tr2 = { 0, CY / 2, CX, CY / 2 + 40 };
        DrawTextW(dc, L"在左侧绘制图案后点击「生成傅里叶动画」", -1, &tr2, DT_CENTER | DT_SINGLELINE | DT_VCENTER);
        SelectObject(dc, of);
        return;
    }
    int showCircles = IsDlgButtonChecked(h, IDC_CHK_CIRCLES) != 0;
    int showSpokes  = IsDlgButtonChecked(h, IDC_CHK_SPOKES) != 0;
    int showTrail   = IsDlgButtonChecked(h, IDC_CHK_TRAIL) != 0;
    int showRef     = IsDlgButtonChecked(h, IDC_CHK_REF) != 0;
    int fade        = IsDlgButtonChecked(h, IDC_CHK_FADE) != 0;

    static Pt chain[MAX_CIRCLES];
    static POINT rp[N_SAMP];
    static POINT tp[N_SAMP + 8];

    for (int si = 0; si < g_animCount; si++) {
        StrokeAnim *a = &g_anims[si];
        if (a->M <= 0) continue;
        int colorIdx = si % 8;
        chain_at(a->circles, a->M, a->n, a->N, chain);
        Pt pen = chain[a->M - 1];
        /* 只在笔画段（相位 < N_stroke）记录轨迹：返回段的扫描不留下线条 */
        if (g_playing && a->n < a->N_stroke &&
            a->trailN < (int)(sizeof(a->trail) / sizeof(a->trail[0])))
            a->trail[a->trailN++] = pen;

        /* 参考路径 */
        if (showRef && a->pathN > 0) {
            for (int i = 0; i < a->pathN; i++) { rp[i].x = rndi(a->path[i].x); rp[i].y = rndi(a->path[i].y); }
            HGDIOBJ op = SelectObject(dc, g_penRef);
            Polyline(dc, rp, a->pathN);
            SelectObject(dc, op);
        }
        /* 圆 */
        if (showCircles) {
            HGDIOBJ op = SelectObject(dc, g_penCircle);
            HGDIOBJ ob = SelectObject(dc, (HGDIOBJ)GetStockObject(NULL_BRUSH));
            for (int i = 0; i < a->M; i++) {
                double rr = a->circles[i].radius;
                if (rr <= 0.4) continue;
                int cx = i ? rndi(chain[i - 1].x) : 0;
                int cy = i ? rndi(chain[i - 1].y) : 0;
                int ri = rndi(rr);
                Ellipse(dc, cx - ri, cy - ri, cx + ri, cy + ri);
            }
            SelectObject(dc, ob);
            SelectObject(dc, op);
        }
        /* 辐条（圆心→末端） */
        if (showSpokes) {
            HGDIOBJ op = SelectObject(dc, g_penSpoke);
            for (int i = 0; i < a->M; i++) {
                int cx = i ? rndi(chain[i - 1].x) : 0;
                int cy = i ? rndi(chain[i - 1].y) : 0;
                MoveToEx(dc, cx, cy, NULL);
                LineTo(dc, rndi(chain[i].x), rndi(chain[i].y));
            }
            SelectObject(dc, op);
        }
        /* 轨迹（实色或渐隐） */
        if (showTrail && a->trailN > 1) {
            for (int i = 0; i < a->trailN; i++) { tp[i].x = rndi(a->trail[i].x); tp[i].y = rndi(a->trail[i].y); }
            if (fade) {
                for (int b = 0; b < FADE_LEVELS; b++) {
                    int i0 = (int)((long long)b * (a->trailN - 1) / (FADE_LEVELS - 1));
                    int i1 = (int)((long long)(b + 1) * (a->trailN - 1) / (FADE_LEVELS - 1));
                    if (i1 <= i0) i1 = i0 + 1;
                    if (i1 >= a->trailN) i1 = a->trailN - 1;
                    if (i1 <= i0) continue;
                    HGDIOBJ op = SelectObject(dc, g_fadePens[colorIdx][b]);
                    Polyline(dc, tp + i0, i1 - i0 + 1);
                    SelectObject(dc, op);
                }
            } else {
                HGDIOBJ op = SelectObject(dc, g_penStroke[colorIdx]);
                Polyline(dc, tp, a->trailN);
                SelectObject(dc, op);
            }
        }
        /* 笔尖 */
        {
            HGDIOBJ ob = SelectObject(dc, g_brushStroke[colorIdx]);
            Ellipse(dc, rndi(pen.x) - 4, rndi(pen.y) - 4, rndi(pen.x) + 4, rndi(pen.y) + 4);
            SelectObject(dc, ob);
            HGDIOBJ op = SelectObject(dc, g_penTipOuter);
            HGDIOBJ ob2 = SelectObject(dc, (HGDIOBJ)GetStockObject(NULL_BRUSH));
            Ellipse(dc, rndi(pen.x) - 8, rndi(pen.y) - 8, rndi(pen.x) + 8, rndi(pen.y) + 8);
            SelectObject(dc, ob2);
            SelectObject(dc, op);
        }
    }
}

/* ============================================================================
   控件创建
   ========================================================================== */
static HWND mk_btn(HWND h, int id, const wchar_t *text, int x, int y, int w) {
    return CreateWindowExW(0, L"BUTTON", text,
        WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP,
        x, y, w, ROW_H, h, (HMENU)(INT_PTR)id, GetModuleHandleW(NULL), NULL);
}
static HWND mk_chk(HWND h, int id, const wchar_t *text, int x, int y, int w, int checked) {
    HWND c = CreateWindowExW(0, L"BUTTON", text,
        WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
        x, y, w, ROW_H, h, (HMENU)(INT_PTR)id, GetModuleHandleW(NULL), NULL);
    CheckDlgButton(h, id, checked ? BST_CHECKED : BST_UNCHECKED);
    return c;
}
static HWND mk_tb(HWND h, int id, int x, int y, int w, int lo, int hi, int val) {
    HWND tb = CreateWindowExW(0, TRACKBAR_CLASSW, L"",
        WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
        x, y, w, ROW_H, h, (HMENU)(INT_PTR)id, GetModuleHandleW(NULL), NULL);
    SendMessageW(tb, TBM_SETRANGE, TRUE, MAKELONG(lo, hi));
    SendMessageW(tb, TBM_SETPOS, TRUE, val);
    return tb;
}
static HWND mk_static(HWND h, int id, const wchar_t *text, int x, int y, int w, DWORD extra) {
    return CreateWindowExW(0, L"STATIC", text,
        WS_CHILD | WS_VISIBLE | SS_LEFT | extra,
        x, y, w, ROW_H, h, (HMENU)(INT_PTR)id, GetModuleHandleW(NULL), NULL);
}

/* ============================================================================
   窗口过程
   ========================================================================== */
static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    switch (m) {
    case WM_CREATE: {
        g_hwnd = h;
        HDC scr = GetDC(NULL);
        g_drawMem = CreateCompatibleDC(scr);
        g_drawBmp = CreateCompatibleBitmap(scr, CX, CY);
        SelectObject(g_drawMem, g_drawBmp);
        g_animMem = CreateCompatibleDC(scr);
        g_animBmp = CreateCompatibleBitmap(scr, CX, CY);
        SelectObject(g_animMem, g_animBmp);
        g_cliMem = CreateCompatibleDC(scr);
        g_cliBmp = CreateCompatibleBitmap(scr, CLIENT_W, CLIENT_H);
        SelectObject(g_cliMem, g_cliBmp);
        ReleaseDC(NULL, scr);
        RECT dr = { 0, 0, CX, CY };
        FillRect(g_drawMem, &dr, (HBRUSH)GetStockObject(WHITE_BRUSH));
        FillRect(g_animMem, &dr, (HBRUSH)GetStockObject(BLACK_BRUSH));
        g_drawRect.left = DRAW_X; g_drawRect.top = DRAW_Y;
        g_drawRect.right = DRAW_X + CX; g_drawRect.bottom = DRAW_Y + CY;
        g_animRect.left = ANIM_X; g_animRect.top = ANIM_Y;
        g_animRect.right = ANIM_X + CX; g_animRect.bottom = ANIM_Y + CY;
        g_brushPanel = CreateSolidBrush(RGB(255, 255, 255));
        {
            RECT full = { 0, 0, CLIENT_W, CLIENT_H };
            FillRect(g_cliMem, &full, g_brushPanel);
        }
        /* 缓存画笔 / 画刷（黑色动画区配色） */
        g_penCircle = CreatePen(PS_SOLID, 1, RGB(110, 140, 190));
        g_penSpoke = CreatePen(PS_SOLID, 1, RGB(90, 100, 120));
        g_penRef = CreatePen(PS_SOLID, 1, RGB(55, 60, 70));
        g_penTipOuter = CreatePen(PS_SOLID, 2, RGB(200, 80, 80));
        g_brushTip = CreateSolidBrush(RGB(255, 107, 107));
        for (int c = 0; c < 8; c++) {
            g_penStroke[c] = CreatePen(PS_SOLID, 2, g_palette[c]);
            g_brushStroke[c] = CreateSolidBrush(g_palette[c]);
            for (int b = 0; b < FADE_LEVELS; b++)
                g_fadePens[c][b] = CreatePen(PS_SOLID, 2,
                    col_lerp(RGB(12, 18, 28), g_palette[c], (double)b / (FADE_LEVELS - 1)));
        }
        g_font = CreateFontW(-15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
            DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
            CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Microsoft YaHei");
        /* 第一行控件 */
        g_btnCompute = mk_btn(h, IDC_BTN_COMPUTE, L"生成傅里叶动画", 12, ROW1_Y, 110);
        g_btnHeart   = mk_btn(h, IDC_BTN_HEART, L"心形示例", 126, ROW1_Y, 80);
        g_btnSpiral  = mk_btn(h, IDC_BTN_SPIRAL, L"螺旋示例", 210, ROW1_Y, 80);
        g_btnUndo    = mk_btn(h, IDC_BTN_UNDO, L"撤销", 294, ROW1_Y, 56);
        g_btnClear   = mk_btn(h, IDC_BTN_CLEAR, L"清空", 354, ROW1_Y, 56);
        g_btnColor   = mk_btn(h, IDC_BTN_COLOR, L"颜色", 414, ROW1_Y, 56);
        mk_static(h, IDC_LBL_PENSIZE, L"粗细", 474, ROW1_Y, 36, 0);
        g_tbPenSize  = mk_tb(h, IDC_TB_PENSIZE, 514, ROW1_Y, 110, 1, 20, 4);
        g_lblPenSize = mk_static(h, 0, L"4", 628, ROW1_Y, 36, 0);
        g_lblDrawInfo = mk_static(h, IDC_LBL_DRAWINFO, L"笔画 0", 690, ROW1_Y, 530, 0);
        /* 第二行控件 */
        g_btnPlay    = mk_btn(h, IDC_BTN_PLAY, L"暂停", 12, ROW2_Y, 88);
        g_btnReset   = mk_btn(h, IDC_BTN_RESET, L"重置", 104, ROW2_Y, 66);
        mk_static(h, 0, L"圈数", 178, ROW2_Y, 38, 0);
        g_tbCircles  = mk_tb(h, IDC_TB_CIRCLES, 220, ROW2_Y, 130, 1, MAX_CIRCLES, 120);
        g_lblCircles = mk_static(h, 0, L"120", 354, ROW2_Y, 40, 0);
        mk_static(h, 0, L"速度", 404, ROW2_Y, 38, 0);
        g_tbSpeed    = mk_tb(h, IDC_TB_SPEED, 446, ROW2_Y, 130, 2, 200, 35);
        g_lblSpeed   = mk_static(h, 0, L"0.35", 580, ROW2_Y, 44, 0);
        mk_chk(h, IDC_CHK_CIRCLES, L"圆", 636, ROW2_Y, 52, 1);
        mk_chk(h, IDC_CHK_SPOKES, L"连线", 692, ROW2_Y, 60, 1);
        mk_chk(h, IDC_CHK_TRAIL, L"轨迹", 756, ROW2_Y, 60, 1);
        mk_chk(h, IDC_CHK_REF, L"参考线", 820, ROW2_Y, 70, 0);
        mk_chk(h, IDC_CHK_FADE, L"渐隐", 894, ROW2_Y, 52, 0);
        g_btnExport = mk_btn(h, IDC_BTN_EXPORT, L"导出参数", 954, ROW2_Y, 104);
        g_btnImport = mk_btn(h, IDC_BTN_IMPORT, L"载入参数", 1066, ROW2_Y, 104);
        g_status = mk_static(h, IDC_STATUS,
            L"提示：在左侧画板绘制图案（可画多个笔画），再点击「生成傅里叶动画」；导出/载入为 JSON 圆参数。",
            12, STATUS_Y, CLIENT_W - 24, SS_END_ELLIPSIS);
        /* 统一字体 */
        HWND ctrls[] = { g_btnCompute, g_btnHeart, g_btnSpiral, g_btnUndo, g_btnClear,
            g_btnColor, g_tbPenSize, g_lblPenSize, g_lblDrawInfo, g_btnPlay, g_btnReset,
            g_tbCircles, g_lblCircles, g_tbSpeed, g_lblSpeed, g_btnExport, g_btnImport, g_status };
        for (int i = 0; i < (int)(sizeof(ctrls) / sizeof(ctrls[0])); i++)
            SendMessageW(ctrls[i], WM_SETFONT, (WPARAM)g_font, TRUE);
        SetTimer(h, 1, 16, NULL);
        return 0;
    }

    case WM_TIMER: {
        static ULONGLONG lastTick = 0;
        ULONGLONG now = GetTickCount64();
        if (lastTick == 0) lastTick = now;
        double dt = (double)(now - lastTick) / 1000.0;
        lastTick = now;
        if (dt > 0.05) dt = 0.05;
        /* 仅在播放且画面真正变化时才重绘；每笔画独立推进相位并在自身周期处清空轨迹 */
        if (g_playing && g_animCount > 0) {
            for (int i = 0; i < g_animCount; i++) {
                StrokeAnim *a = &g_anims[i];
                a->n += a->N * g_speed * dt;
                if (a->n >= a->N) { a->n -= a->N; a->trailN = 0; }
            }
            InvalidateRect(h, &g_animRect, FALSE);
            UpdateWindow(h);
        }
        return 0;
    }

    case WM_LBUTTONDOWN: {
        int mx = (int)(short)LOWORD(l), my = (int)(short)HIWORD(l);
        if (mx >= DRAW_X && mx < DRAW_X + CX && my >= DRAW_Y && my < DRAW_Y + CY) {
            g_drawing = 1;
            SetCapture(h);
            g_curN = 0;
            Pt p;
            p.x = clampd((double)(mx - DRAW_X), 0, CX - 1);
            p.y = clampd((double)(my - DRAW_Y), 0, CY - 1);
            cur_push(p);
            stroke_dot(p.x, p.y, g_penSize, g_penColor);
            /* 只失效笔点的小区域，避免整块画布重绘 */
            RECT upd;
            int r = g_penSize / 2 + 2;
            upd.left = (int)p.x - r; upd.top = (int)p.y - r;
            upd.right = (int)p.x + r + 1; upd.bottom = (int)p.y + r + 1;
            IntersectRect(&upd, &upd, &g_drawRect);
            InvalidateRect(h, &upd, FALSE);
        }
        return 0;
    }
    case WM_MOUSEMOVE: {
        if (g_drawing && (w & MK_LBUTTON)) {
            int mx = (int)(short)LOWORD(l), my = (int)(short)HIWORD(l);
            Pt p;
            p.x = clampd((double)(mx - DRAW_X), 0, CX - 1);
            p.y = clampd((double)(my - DRAW_Y), 0, CY - 1);
            Pt last = g_cur[g_curN - 1];
            if (dist(p.x - last.x, p.y - last.y) >= 0.5) {
                cur_push(p);
                stroke_line(last, p, g_penSize, g_penColor);
                /* 只失效本段线段的包围盒，画画时每帧只重绘一小块 */
                RECT upd;
                int r = g_penSize / 2 + 2;
                upd.left = (int)(last.x < p.x ? last.x : p.x) - r;
                upd.top = (int)(last.y < p.y ? last.y : p.y) - r;
                upd.right = (int)(last.x > p.x ? last.x : p.x) + r + 1;
                upd.bottom = (int)(last.y > p.y ? last.y : p.y) + r + 1;
                IntersectRect(&upd, &upd, &g_drawRect);
                InvalidateRect(h, &upd, FALSE);
            }
        }
        return 0;
    }
    case WM_LBUTTONUP: {
        if (g_drawing) {
            g_drawing = 0;
            ReleaseCapture();
            if (g_curN > 0) {
                Stroke st;
                st.pts = g_cur; st.n = g_curN; st.cap = g_curCap;
                st.color = g_penColor; st.size = g_penSize;
                strokes_push(&st);
                g_cur = NULL; g_curN = 0; g_curCap = 0;
                update_draw_info();
            }
            InvalidateRect(h, &g_drawRect, FALSE);
        }
        return 0;
    }

    case WM_COMMAND: {
        int id = LOWORD(w);
        switch (id) {
        case IDC_BTN_COMPUTE: compute(h); break;
        case IDC_BTN_HEART: { int n; Pt *p = demo_heart(&n); load_demo(h, p, n); break; }
        case IDC_BTN_SPIRAL: { int n; Pt *p = demo_spiral(&n); load_demo(h, p, n); break; }
        case IDC_BTN_UNDO: undo_stroke(); break;
        case IDC_BTN_CLEAR: clear_all(h); break;
        case IDC_BTN_COLOR: {
            CHOOSECOLORW cc;
            memset(&cc, 0, sizeof(cc));
            cc.lStructSize = sizeof(cc);
            cc.hwndOwner = h;
            cc.lpCustColors = g_custom;
            cc.rgbResult = g_penColor;
            cc.Flags = CC_RGBINIT | CC_FULLOPEN;
            if (ChooseColorW(&cc)) g_penColor = cc.rgbResult;
            break;
        }
        case IDC_BTN_PLAY:
            if (g_animCount <= 0) set_status(L"还没有可播放的动画，请先生成");
            else { g_playing = !g_playing; set_play_label(); }
            break;
        case IDC_BTN_RESET:
            for (int i = 0; i < g_animCount; i++) { g_anims[i].n = 0; g_anims[i].trailN = 0; }
            InvalidateRect(h, &g_animRect, FALSE);
            break;
        case IDC_BTN_EXPORT: do_export(h); break;
        case IDC_BTN_IMPORT: do_import(h); break;
        default:
            if (id >= IDC_CHK_CIRCLES && id <= IDC_CHK_FADE)
                InvalidateRect(h, &g_animRect, FALSE);
            break;
        }
        return 0;
    }

    case WM_HSCROLL: {
        HWND tb = (HWND)l;
        if (!tb) break;
        int id = GetDlgCtrlID(tb);
        int pos = (int)SendMessageW(tb, TBM_GETPOS, 0, 0);
        wchar_t b[16];
        if (id == IDC_TB_CIRCLES) {
            g_circleCount = pos;
            swprintf(b, 16, L"%d", pos);
            SetWindowTextW(g_lblCircles, b);
            if (g_animCount > 0) {
                rebuild_stroke_anims();
                for (int i = 0; i < g_animCount; i++) g_anims[i].n = 0;
            }
            InvalidateRect(h, &g_animRect, FALSE);
        } else if (id == IDC_TB_SPEED) {
            g_speed = pos / 100.0;
            swprintf(b, 16, L"%.2f", g_speed);
            SetWindowTextW(g_lblSpeed, b);
        } else if (id == IDC_TB_PENSIZE) {
            g_penSize = pos;
            swprintf(b, 16, L"%d", pos);
            SetWindowTextW(g_lblPenSize, b);
        }
        return 0;
    }

    case WM_SETCURSOR:
        if (LOWORD(l) == HTCLIENT) {
            POINT pt;
            GetCursorPos(&pt);
            ScreenToClient(h, &pt);
            if (pt.x >= DRAW_X && pt.x < DRAW_X + CX && pt.y >= DRAW_Y && pt.y < DRAW_Y + CY) {
                SetCursor(LoadCursorW(NULL, IDC_CROSS));
                return TRUE;
            }
        }
        break;

    case WM_KEYDOWN:
        if (w == VK_SPACE && GetFocus() == h) {
            if (g_animCount > 0) { g_playing = !g_playing; set_play_label(); }
            return 0;
        }
        break;

    case WM_ERASEBKGND:
        return 1;

    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc = BeginPaint(h, &ps);
        RECT paint = ps.rcPaint, dummy;
        /* 按失效区域判断需要更新的画布：画画时只有绘制区失效，动画区不必重渲染 */
        int needDraw = IntersectRect(&dummy, &paint, &g_drawRect) != 0;
        int needAnim = IntersectRect(&dummy, &paint, &g_animRect) != 0;
        /* 整窗合成（全部写入内存 g_cliMem，屏幕完全不动）：
           耗时渲染 → 背景 → 画布 → 动画 → 边框 */
        if (needAnim) render_animation(g_animMem, h);
        FillRect(g_cliMem, &paint, g_brushPanel);
        if (needDraw) BitBlt(g_cliMem, DRAW_X, DRAW_Y, CX, CY, g_drawMem, 0, 0, SRCCOPY);
        if (needAnim) BitBlt(g_cliMem, ANIM_X, ANIM_Y, CX, CY, g_animMem, 0, 0, SRCCOPY);
        if (needDraw || needAnim) {
            HPEN p = CreatePen(PS_SOLID, 2, RGB(140, 152, 168));
            HGDIOBJ op = SelectObject(g_cliMem, p);
            HGDIOBJ ob = SelectObject(g_cliMem, (HGDIOBJ)GetStockObject(NULL_BRUSH));
            if (needDraw) Rectangle(g_cliMem, g_drawRect.left, g_drawRect.top,
                                    g_drawRect.right, g_drawRect.bottom);
            SelectObject(g_cliMem, ob);
            SelectObject(g_cliMem, op);
            DeleteObject(p);
            HBRUSH bd = CreateSolidBrush(RGB(214, 220, 228));
            if (needAnim) FrameRect(g_cliMem, &g_animRect, bd);
            DeleteObject(bd);
        }
        /* 一次 BitBlt 呈现失效区：窗口表面每次绘制只更新一次，杜绝中间态闪烁 */
        BitBlt(dc, paint.left, paint.top, paint.right - paint.left, paint.bottom - paint.top,
               g_cliMem, paint.left, paint.top, SRCCOPY);
        EndPaint(h, &ps);
        return 0;
    }

    case WM_DESTROY:
        KillTimer(h, 1);
        for (int i = 0; i < g_strokes.n; i++) free(g_strokes.s[i].pts);
        free(g_strokes.s);
        if (g_cur) free(g_cur);
        if (g_drawBmp) DeleteObject(g_drawBmp);
        if (g_drawMem) DeleteDC(g_drawMem);
        if (g_animBmp) DeleteObject(g_animBmp);
        if (g_animMem) DeleteDC(g_animMem);
        if (g_cliBmp) DeleteObject(g_cliBmp);
        if (g_cliMem) DeleteDC(g_cliMem);
        if (g_penCircle) DeleteObject(g_penCircle);
        if (g_penSpoke) DeleteObject(g_penSpoke);
        if (g_penRef) DeleteObject(g_penRef);
        if (g_penTipOuter) DeleteObject(g_penTipOuter);
        if (g_brushPanel) DeleteObject(g_brushPanel);
        if (g_brushTip) DeleteObject(g_brushTip);
        for (int c = 0; c < 8; c++) {
            if (g_penStroke[c]) DeleteObject(g_penStroke[c]);
            if (g_brushStroke[c]) DeleteObject(g_brushStroke[c]);
            for (int b = 0; b < FADE_LEVELS; b++)
                if (g_fadePens[c][b]) DeleteObject(g_fadePens[c][b]);
        }
        if (g_font) DeleteObject(g_font);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(h, m, w, l);
}

/* ============================================================================
   自测（无界面）：fourier-drawing.exe --selftest
   ========================================================================== */
static int run_selftest(void) {
    wchar_t dir[MAX_PATH];
    GetModuleFileNameW(NULL, dir, MAX_PATH);
    wchar_t *s = wcsrchr(dir, L'\\');
    if (s) *s = 0;
    wchar_t fpath[MAX_PATH + 32];
    swprintf(fpath, MAX_PATH + 32, L"%ls\\selftest.txt", dir);
    FILE *log = _wfopen(fpath, L"wb");
    if (!log) { /* 降级：当前目录 */
        wchar_t cur[MAX_PATH];
        GetCurrentDirectoryW(MAX_PATH, cur);
        swprintf(fpath, MAX_PATH + 32, L"%ls\\selftest.txt", cur);
        log = _wfopen(fpath, L"wb");
    }
    if (!log) { /* 降级：CreateFileW */
        HANDLE hf = CreateFileW(fpath, GENERIC_WRITE, FILE_SHARE_READ, NULL,
                                CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hf != INVALID_HANDLE_VALUE) {
            int fd = _open_osfhandle((intptr_t)hf, _O_TEXT);
            log = _fdopen(fd, "w");
        }
    }
    if (!log) return 2;
    int ok = 1;
#define T(c, m) do { \
        char mb[512]; \
        WideCharToMultiByte(CP_UTF8, 0, m, -1, mb, 512, NULL, NULL); \
        if (c) { fprintf(log, "PASS: %s\n", mb); } \
        else { fprintf(log, "FAIL: %s\n", mb); ok = 0; } \
    } while (0)

    /* 1) 圆形 DFT：c0=圆心、c1=半径、c_{-1}≈0，2 个圆精确重建 */
    {
        int N = 256; Pt pts[256];
        double R = 50, ctr = 100;
        for (int n = 0; n < N; n++) {
            double a = 2.0 * M_PI * n / N;
            pts[n].x = ctr + R * cos(a);
            pts[n].y = ctr + R * sin(a);
        }
        Coeff cs[256];
        dft(pts, N, cs);
        const Coeff *c0 = NULL, *c1 = NULL, *cn = NULL;
        for (int i = 0; i < N; i++) {
            if (cs[i].k == 0) c0 = &cs[i];
            if (cs[i].k == 1) c1 = &cs[i];
            if (cs[i].k == -1) cn = &cs[i];
        }
        T(c0 && fabs(c0->re - 100) < 1e-6 && fabs(c0->im - 100) < 1e-6, L"圆形 c0=圆心 (100,100)");
        T(c1 && fabs(c1->re - 50) < 1e-6 && fabs(c1->im) < 1e-6, L"圆形 c1=半径 50");
        T(cn && cn->mag < 1e-6, L"圆形 c_{-1}≈0");
        T(cs[0].k == 0 && cs[1].k == 1, L"系数按模长降序排列");
        Circle cir[2];
        for (int i = 0; i < 2; i++) {
            cir[i].k = cs[i].k; cir[i].re = cs[i].re; cir[i].im = cs[i].im;
            cir[i].radius = cs[i].mag; cir[i].phase = atan2(cs[i].im, cs[i].re);
        }
        double err = approx_error(pts, N, cir, 2, N);
        T(err < 1e-6, L"2 个圆重建圆形误差≈0");
        Pt ch[2], ch2[2];
        chain_at(cir, 2, 0, N, ch);
        chain_at(cir, 2, (double)N, N, ch2);
        T(fabs(ch[1].x - ch2[1].x) < 1e-9 && fabs(ch[1].y - ch2[1].y) < 1e-9, L"n=0 与 n=N 周期闭合");
    }

    /* 2) 心形：300 个圆重建误差 < 1 */
    {
        Pt raw[600];
        for (int i = 0; i < 600; i++) {
            double t = 2.0 * M_PI * i / 600;
            double sn = sin(t);
            raw[i].x = 16.0 * sn * sn * sn;
            raw[i].y = 13.0 * cos(t) - 5.0 * cos(2 * t) - 2.0 * cos(3 * t) - cos(4 * t);
        }
        Pt rp[2048];
        resample(raw, 600, rp, 2048);
        Coeff full[2048];
        dft(rp, 2048, full);
        Circle cir[300];
        for (int i = 0; i < 300; i++) {
            cir[i].k = full[i].k; cir[i].re = full[i].re; cir[i].im = full[i].im;
            cir[i].radius = full[i].mag; cir[i].phase = atan2(full[i].im, full[i].re);
        }
        double err = approx_error(rp, 2048, cir, 300, 2048);
        {
            wchar_t wb[128];
            swprintf(wb, 128, L"心形 300 圆平均误差: %.4f", err);
            char mb[256];
            WideCharToMultiByte(CP_UTF8, 0, wb, -1, mb, 256, NULL, NULL);
            fprintf(log, "%s\n", mb);
        }
        T(err < 1, L"心形 300 圆误差 < 1");
    }

    /* 3) 直线重采样均匀 */
    {
        Pt line[2] = { { 0, 0 }, { 100, 0 } };
        Pt rp[100];
        resample(line, 2, rp, 100);
        T(fabs(rp[1].x - rp[0].x - 1) < 1e-9, L"直线采样间距≈1");
        T(fabs(rp[0].x) < 1e-9 && fabs(rp[99].x - 99) < 1e-9, L"直线端点正确");
    }

    /* 4) 多笔画完整管线：心形+螺旋 → v2 导出 → 导入 → 校验 */
    {
        StrokeAnim a1, a2;
        memset(&a1, 0, sizeof(a1));
        memset(&a2, 0, sizeof(a2));
        int n1;
        Pt *hp = demo_heart(&n1);
        build_anim(&a1, hp, n1, 120);
        int n2;
        Pt *sp = demo_spiral(&n2);
        build_anim(&a2, sp, n2, 120);
        free(hp); free(sp);
        StrokeAnim two[2] = { a1, a2 };
        wchar_t jp[MAX_PATH + 32];
        swprintf(jp, MAX_PATH + 32, L"%ls\\roundtrip2.json", dir);
        FILE *f = _wfopen(jp, L"wb");
        json_write_strokes(f, two, 2);
        fclose(f);
        f = _wfopen(jp, L"rb");
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        char *buf = (char *)malloc((size_t)sz + 1);
        fread(buf, 1, (size_t)sz, f);
        fclose(f);
        buf[sz] = 0;
        wchar_t err[128];
        int ok2 = import_data(buf, err, 128);
        T(ok2, L"v2 多笔画导入成功");
        if (ok2) {
            T(g_animCount == 2, L"v2 导入得到 2 个笔画");
            T(g_anims[0].M == 120 && g_anims[1].M == 120, L"每笔 120 个圆");
            T(g_anims[0].pathN == 2048 && g_anims[1].pathN == 2048, L"每笔路径 2048 点");
            T(g_anims[0].N_stroke == 2048 && g_anims[0].N == 2048 + N_EXT, L"闭合周期正确");
            double e0 = approx_error(g_anims[0].path, g_anims[0].N_stroke,
                                     g_anims[0].circles, g_anims[0].M, g_anims[0].N);
            T(e0 < 1, L"导入后心形重建误差 < 1");
            /* 导入后系数与导出前一致（6 位小数舍入误差内） */
            T(g_anims[0].circles[0].k == a1.circles[0].k &&
              fabs(g_anims[0].circles[0].re - a1.circles[0].re) < 1e-4 &&
              fabs(g_anims[0].circles[0].im - a1.circles[0].im) < 1e-4 &&
              fabs(g_anims[0].circles[0].radius - a1.circles[0].radius) < 1e-4,
              L"导入后系数与导出前一致");
        }
        free(buf);
        _wremove(jp);
        a_free_all();
    }

    /* 5) v1 兼容：满规模（400 圆 + 2048 路径 + 中文 description） */
    {
        StrokeAnim a;
        memset(&a, 0, sizeof(a));
        int n;
        Pt *hp = demo_heart(&n);
        resample(hp, n, a.path, 2048);
        a.pathN = 2048; a.N = 2048; a.fullN = 2048;
        dft(a.path, 2048, a.full);
        a.M = 400;
        for (int i = 0; i < a.M; i++) {
            a.circles[i].k = a.full[i].k; a.circles[i].re = a.full[i].re;
            a.circles[i].im = a.full[i].im; a.circles[i].radius = a.full[i].mag;
            a.circles[i].phase = atan2(a.full[i].im, a.full[i].re);
        }
        free(hp);
        wchar_t jp[MAX_PATH + 32];
        swprintf(jp, MAX_PATH + 32, L"%ls\\roundtrip1.json", dir);
        FILE *f = _wfopen(jp, L"wb");
        fputs("{\n  \"app\":\"fourier-drawing\",\n  \"version\":1,\n"
              "  \"description\":\"傅里叶画板导出的圆参数：k=频率，radius=半径，phase=初始相位，re/im=复系数。\",\n"
              "  \"sampleCount\":2048,\n  \"circles\":[\n", f);
        for (int i = 0; i < a.M; i++)
            fprintf(f, "    {\"k\":%d,\"radius\":%.6f,\"phase\":%.6f,\"re\":%.6f,\"im\":%.6f}%s\n",
                    a.circles[i].k, a.circles[i].radius, a.circles[i].phase,
                    a.circles[i].re, a.circles[i].im, i < a.M - 1 ? "," : "");
        fputs("  ],\n  \"path\":[\n", f);
        for (int i = 0; i < a.pathN; i++)
            fprintf(f, "    [%.2f,%.2f]%s\n", a.path[i].x, a.path[i].y, i < a.pathN - 1 ? "," : "");
        fputs("  ]\n}\n", f);
        fclose(f);
        f = _wfopen(jp, L"rb");
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        char *buf = (char *)malloc((size_t)sz + 1);
        fread(buf, 1, (size_t)sz, f);
        fclose(f);
        buf[sz] = 0;
        wchar_t err[128];
        int ok3 = import_data(buf, err, 128);
        T(ok3, L"v1 满规模导入成功（无闪退）");
        if (ok3) {
            T(g_animCount == 1, L"v1 导入得到 1 个笔画");
            T(g_anims[0].M == 400, L"v1 导入 400 个圆");
            T(g_anims[0].pathN == 2048, L"v1 导入路径 2048 点");
            double e0 = approx_error(g_anims[0].path, g_anims[0].N_stroke,
                                     g_anims[0].circles, g_anims[0].M, g_anims[0].N);
            T(e0 < 1, L"v1 导入后重建误差 < 1");
        }
        free(buf);
        _wremove(jp);
        a_free_all();
    }

    /* 6) UTF-8 中文键值 */
    {
        JV *zh = json_parse("{\"名字\":\"傅里叶\"}");
        T(zh != NULL, L"UTF-8 中文解析");
        if (zh) {
            JV *v = jv_find(zh, "名字");
            T(v && v->type == JT_STR && wcscmp(v->str, L"傅里叶") == 0, L"中文键值正确");
            a_free_all();
        }
    }

    /* 7) 回归：生成 → 清空 → 再生成，第二次必须显示新笔画（g_animCount 更新顺序） */
    {
        /* 第一次：心形 */
        int n1;
        Pt *hp = demo_heart(&n1);
        Stroke st1;
        st1.pts = hp; st1.n = n1; st1.cap = n1; st1.color = RGB(17, 24, 39); st1.size = 4;
        for (int i = 0; i < g_strokes.n; i++) free(g_strokes.s[i].pts);
        g_strokes.n = 0;
        strokes_push(&st1);
        compute(NULL);
        T(g_animCount == 1, L"第一次生成：1 个笔画");
        double heartC1re = g_anims[0].circles[1].re;
        /* 清空（模拟 clear_all 的动画部分） */
        g_animCount = 0;
        /* 第二次：螺旋 */
        int n2;
        Pt *sp = demo_spiral(&n2);
        Stroke st2;
        st2.pts = sp; st2.n = n2; st2.cap = n2; st2.color = RGB(17, 24, 39); st2.size = 4;
        for (int i = 0; i < g_strokes.n; i++) free(g_strokes.s[i].pts);
        g_strokes.n = 0;
        strokes_push(&st2);
        compute(NULL);
        T(g_animCount == 1, L"第二次生成：1 个笔画");
        StrokeAnim *a = &g_anims[0];
        /* 修复后：circles 由新 full 重建；修复前：残留第一次（心形）的旧系数 */
        T(fabs(a->circles[1].re - a->full[1].re) < 1e-9, L"再生成：circles 与新 full 一致（非旧数据）");
        T(fabs(a->circles[1].re - heartC1re) > 1.0, L"再生成：circles 确实来自新笔画");
        double e2 = approx_error(a->path, a->N_stroke, a->circles, a->M, a->N);
        T(e2 < 5, L"再生成：重建误差正常（旧数据会严重失配）");
        /* 清理 */
        for (int i = 0; i < g_strokes.n; i++) free(g_strokes.s[i].pts);
        g_strokes.n = 0;
    }

    /* 8) 回归：开放笔画经闭合后，轨迹不再出现异常长线段（吉布斯振荡） */
    {
        StrokeAnim a;
        memset(&a, 0, sizeof(a));
        Pt arc[201];
        for (int i = 0; i <= 200; i++) {
            double t = (double)i / 200, th = M_PI + M_PI * t;
            arc[i].x = 300 + 200 * cos(th);
            arc[i].y = 300 + 200 * sin(th);
        }
        build_anim(&a, arc, 201, 120);
        /* 模拟动画：逐帧推进相位，笔画段记录轨迹，返回段跳过，换圈清空 */
        double maxSeg = 0;
        for (int f = 0; f < 300; f++) {
            a.n += a.N * 0.35 * 0.016;
            if (a.n >= a.N) { a.n -= a.N; a.trailN = 0; }
            double x = 0, y = 0;
            for (int j = 0; j < a.M; j++) {
                double ang = a.circles[j].phase + 2.0 * M_PI * a.circles[j].k * a.n / a.N;
                x += a.circles[j].radius * cos(ang);
                y += a.circles[j].radius * sin(ang);
            }
            Pt pen = { x, y };
            if (a.n < a.N_stroke && a.trailN < (int)(sizeof(a.trail) / sizeof(a.trail[0]))) {
                if (a.trailN > 0) {
                    double L = dist(pen.x - a.trail[a.trailN - 1].x, pen.y - a.trail[a.trailN - 1].y);
                    if (L > maxSeg) maxSeg = L;
                }
                a.trail[a.trailN++] = pen;
            }
        }
        {
            wchar_t wb[128];
            swprintf(wb, 128, L"闭合后最大轨迹段长: %.1f px", maxSeg);
            char mb[256];
            WideCharToMultiByte(CP_UTF8, 0, wb, -1, mb, 256, NULL, NULL);
            fprintf(log, "%s\n", mb);
        }
        T(maxSeg < 10, L"闭合后无异常长轨迹段（吉布斯振荡消除）");
    }

    /* 9) 机会性回归：目录中的真实导出文件（若有）导入验证 */
    {
        wchar_t pat[MAX_PATH + 32];
        swprintf(pat, MAX_PATH + 32, L"%ls\\fourier-circles-*.json", dir);
        WIN32_FIND_DATAW fd;
        HANDLE hf = FindFirstFileW(pat, &fd);
        if (hf != INVALID_HANDLE_VALUE) {
            wchar_t jp[MAX_PATH + 64];
            swprintf(jp, MAX_PATH + 64, L"%ls\\%ls", dir, fd.cFileName);
            FILE *f = _wfopen(jp, L"rb");
            if (f) {
                fseek(f, 0, SEEK_END);
                long sz = ftell(f);
                fseek(f, 0, SEEK_SET);
                char *buf = (char *)malloc((size_t)sz + 1);
                fread(buf, 1, (size_t)sz, f);
                fclose(f);
                buf[sz] = 0;
                wchar_t err[128];
                int ok9 = import_data(buf, err, 128);
                T(ok9, L"真实导出文件导入成功");
                if (ok9) {
                    T(g_animCount >= 1, L"真实文件至少 1 个笔画");
                    double eSum = 0;
                    for (int i = 0; i < g_animCount; i++)
                        eSum += approx_error(g_anims[i].path, g_anims[i].N_stroke,
                                             g_anims[i].circles, g_anims[i].M, g_anims[i].N);
                    T(eSum / g_animCount < 5, L"真实文件重建误差正常");
                }
                free(buf);
                a_free_all();
            }
            FindClose(hf);
        } else {
            fprintf(log, "SKIP: 目录中没有 fourier-circles-*.json 导出文件\n");
        }
    }

    fprintf(log, ok ? "\nALL TESTS PASSED\n" : "\nSOME TESTS FAILED\n");
    fclose(log);
    return ok ? 0 : 1;
#undef T
}

/* ============================================================================
   入口
   ========================================================================== */
int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE hPrev, PWSTR pCmdLine, int nShow) {
    (void)hPrev;
    setlocale(LC_NUMERIC, "C");
    if (wcsstr(pCmdLine, L"--selftest")) return run_selftest();

    INITCOMMONCONTROLSEX icc;
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_BAR_CLASSES;
    InitCommonControlsEx(&icc);

    WNDCLASSW wc;
    memset(&wc, 0, sizeof(wc));
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = L"FourierDrawing";
    wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon = LoadIconW(NULL, IDI_APPLICATION);
    wc.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
    if (!RegisterClassW(&wc)) return 1;

    RECT rc = { 0, 0, CLIENT_W, CLIENT_H };
    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_CLIPCHILDREN;
    AdjustWindowRect(&rc, style, FALSE);
    HWND h = CreateWindowExW(0, L"FourierDrawing", L"傅里叶画板 · Fourier Epicycles",
        style, CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right - rc.left, rc.bottom - rc.top, NULL, NULL, hInst, NULL);
    if (!h) return 1;
    ShowWindow(h, nShow);
    UpdateWindow(h);

    MSG msg;
    while (GetMessageW(&msg, NULL, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return 0;
}
