
#include "camera_process.hpp"


// Constants for mode selection

// Function declarations
void printBanner();
void handleMode(MODE mode, pcie_trans &pcie_trans_t);
MODE parseArguments(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    // Initialize PCIe transaction structure
    pcie_trans pcie_trans_t;

    // Print application banner
    printBanner();

    // Parse command-line arguments to determine the mode
    MODE mode = parseArguments(argc, argv);

    // Configure the PCIe transaction
    configure(&pcie_trans_t);
    printf("Configuration successful\n");

    // Handle the selected mode 
    handleMode(mode, pcie_trans_t);

    return 0;
}

void printBanner()
{
    printf(ANSI_COLOR_BLUE);
    printf("************************************************\n");
    printf("*******                              ***********\n");
    printf("*******  CIS, DVS Video Application  ***********\n");
    printf("*******                              ***********\n");
    printf("************************************************\n");
    printf(ANSI_COLOR_RESET);
}

void handleMode(MODE mode, pcie_trans &pcie_trans_t)
{
    switch (mode)
    {
    case CIS_ONLY_DISPLAY:
        CIS_only_display_stream(&pcie_trans_t);
        break;
    case DVS_ONLY_DISPLAY:
        DVS_only_display_stream(&pcie_trans_t);
        break;
    case DVS_ONLY_CHECK:
        printf("currently on DVS_ONLY_CHECK mode\r\n");
        DVS_only_check_frame(&pcie_trans_t);
        break;
    case CIS_DVS:
        printf("currently on CIS, DVS display mode\r\n");
        CIS_DVS_stream(&pcie_trans_t);
        break;
    case DVS_STORE:
        printf("currently on DVS store mode\r\n");
        DVS_threading(&pcie_trans_t);
        break;
    case DVS_ROI:
        printf("currently on DVS ROI detection mode\r\n");
        DVS_crop_coord(&pcie_trans_t);
        break;
    case CIS_DVS_BBOX:
        printf("currently on CIS, DVS crop mode\r\n");
        CIS_DVS_bbox_stream(&pcie_trans_t);
    default:
        fprintf(stderr, "Error: Unknown mode\n");
        exit(EXIT_FAILURE);
    }
}

MODE parseArguments(int argc, char *argv[])
{
    MODE mode = DVS_ONLY_DISPLAY; // Default mode

    // Define command-line options
    struct option long_options[] = {
        {"cis", no_argument, nullptr, 'c'},
        {"dvs", no_argument, nullptr, 'd'},
        {"check", no_argument, nullptr, 'x'},
        {"cis-dvs", no_argument, nullptr, 's'},
        {"write-dvs", no_argument, nullptr, 'w'},
        {"roi", no_argument, nullptr, 'r'},
        {"bbox", no_argument, nullptr, 'b'},
        {nullptr, 0, nullptr, 0}};

    // Parse command-line arguments
    int opt;
    while ((opt = getopt_long(argc, argv, "cdxswrb", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
        case 'c':
            mode = CIS_ONLY_DISPLAY;
            break;
        case 'd':
            mode = DVS_ONLY_DISPLAY;
            break;
        case 'x':
            mode = DVS_ONLY_CHECK;
            break;
        case 's':
            mode = CIS_DVS;
            break;
        case 'w':
            mode = DVS_STORE;
            break;
        case 'r':
            mode = DVS_ROI;
            break;
        case 'b':
            mode = CIS_DVS_BBOX;
            break;
        default:
            fprintf(stderr, "Usage: %s [--cis | --dvs | --check | --cis-dvs | --write-dvs | --roi | --bbox]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    return mode;
}
