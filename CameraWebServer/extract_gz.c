#include <stdio.h>
#include <stdlib.h>

// Include the byte array
#include "index_ov2640_html_gz.c"

int main() {
    FILE *fp = fopen("index_ov2640.html.gz", "wb");
    if (!fp) {
        perror("Cannot open file");
        return 1;
    }

    fwrite(index_ov2640_html_gz, 1, index_ov2640_html_gz_len, fp);
    fclose(fp);

    printf("File extracted: index_ov2640.html.gz\n");
    return 0;
}

