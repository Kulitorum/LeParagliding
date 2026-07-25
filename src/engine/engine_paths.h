#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void lep_configure_paths(const char *inputPath, const char *outputDirectory);
char *lep_input_path(void);
char *lep_output_path(const char *fileName);
char *lep_output_path_fortran(const char *fileName, int length);
void lep_ensure_output_subdirectory(const char *directoryName);
int lep_path_length(const char *path);
int lep_count_fields(const char *text, int length);
int lep_parse_planform_row(
    const char *text,
    int length,
    double *ribNumber,
    double *xRib,
    double *leadingEdge,
    double *trailingEdge,
    double *xPrime,
    double *z,
    double *beta,
    double *rotationPoint,
    double *washin,
    double *tipAngle1,
    double *tipAngle2);

#ifdef __cplusplus
}
#endif
